#pragma once

// IF-0002's content policy, expressed as pure logic: per-category severity
// floors (IF-0002 section 10.3 and 11.4), repeat suppression (A3), and the
// per-category token bucket (A4).
//
// **SDK-free and FreeRTOS-free, deliberately.** Every rule in here is a policy
// decision that can be inverted, made vacuous, or quietly loosened by a
// refactor with no bench session noticing -- the same argument bootloader.hpp
// makes for its gate. It is compiled directly into the host tests. log.cpp owns
// the UART, the ring and the critical section; this file owns none of them and
// must not learn about them.
//
// **A4 is load-bearing for safety, not tidiness.** IF-0002 section 10.5 point 4:
// log_console::push() takes taskENTER_CRITICAL(), which on this SMP port is a
// genuine spinlock pair (rt.h), so a flood of log lines from a core 0 task
// contends the same spinlock the control task on core 1 needs. The bucket is
// what bounds that contention, and it is the reason the runtime log level of
// IF-0002 section 10 can be allowed to exist at all. Weakening the bucket
// weakens a control-loop property, not a formatting preference.

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace log_policy
{

// One line, matching log.cpp's formatting buffer. The repeat suppressor stores
// a whole line, so this also sizes its comparison buffer.
inline constexpr size_t line_max = 192;

// Severity. Ordered so a floor is a comparison rather than a set test, which is
// the same reason LAWN_LOG_LEVEL in the dialect is ordered.
//
// `off` is a floor value only; nothing is ever emitted AT it. Snapshot (`S`) and
// boot (`B`) lines are not severities and are not filtered by a floor at all --
// a state snapshot you can switch off is not a state snapshot.
enum class Level : uint8_t
{
    trace = 0,
    debug = 1,
    info = 2,
    warn = 3,
    error = 4,
    off = 5,
};

// IF-0002 section 4's fixed category set, plus `boot` for the B block and
// `other` for anything unrecognised.
//
// **An unrecognised category is NEVER dropped.** It lands in `other` and
// increments a counter, so a typo costs a bucket slot and shows up in a number
// rather than silently losing the line someone wrote. Dropping would make a
// misspelling indistinguishable from a component that has gone quiet, which is
// the exact confusion this whole document exists to prevent.
enum class Cat : uint8_t
{
    log = 0,
    sys,
    saf,
    whl,
    pid,
    mot,
    imu,
    lnk,
    boot,
    other,
};

inline constexpr size_t cat_count = static_cast<size_t>(Cat::other) + 1;

constexpr char levelChar(Level level)
{
    switch (level)
    {
    case Level::trace: return 'T';
    case Level::debug: return 'D';
    case Level::info: return 'I';
    case Level::warn: return 'W';
    case Level::error: return 'E';
    default: return '?';
    }
}

// Resolves a call site's category token to its bucket.
//
// A token may be `slot` or `slot.sub` -- `imu.i2c`, `imu.drdy`, `whl.enc`. The
// **printed** category keeps the full token so `grep imu.drdy` still narrows to
// one driver; the **bucket and the floor** key on the slot, so one subsystem
// shares one budget. That split is why the drivers did not all have to collapse
// onto three-letter names: resolution for the reader, coarse accounting for the
// limiter.
inline Cat resolveCat(const char* token)
{
    if (token == nullptr)
    {
        return Cat::other;
    }

    // Length of the slot: up to the '.' or the terminator.
    size_t n = 0;
    while (token[n] != '\0' && token[n] != '.')
    {
        n++;
    }

    struct Entry
    {
        const char* name;
        Cat cat;
    };
    // `other` is deliberately absent: it is the fallback, not a spellable name.
    static constexpr Entry table[] = {
        {"log", Cat::log}, {"sys", Cat::sys}, {"saf", Cat::saf},
        {"whl", Cat::whl}, {"pid", Cat::pid}, {"mot", Cat::mot},
        {"imu", Cat::imu}, {"lnk", Cat::lnk}, {"boot", Cat::boot},
    };

    for (const Entry& e : table)
    {
        if (strlen(e.name) == n && strncmp(token, e.name, n) == 0)
        {
            return e.cat;
        }
    }
    return Cat::other;
}

// --------------------------------------------------------------------------
// A3 -- repeat suppression
// --------------------------------------------------------------------------

// Collapses a run of identical lines into one line plus a count.
//
// **Compares whole line bytes, not a hash.** A hash collision would suppress a
// DIFFERENT line, and a line that vanishes for a reason no counter explains is
// worse than the ~192 bytes of storage an exact comparison costs. At this size
// the memcmp is the same order as the memcpy log.cpp already does under the
// lock.
//
// A3 exists because A2 (edge-triggered events) WILL be violated by a
// well-meaning change. It is the backstop, not the mechanism -- a design that
// relied on it would be logging conditions and counting on the suppressor to
// hide them, which loses the timing information an edge would have carried.
class RepeatSuppressor
{
  public:
    struct Decision
    {
        // Emit the line the caller is holding.
        bool emit_line;
        // If non-zero, emit a `repeated n=<flush_count>` line FIRST. The run it
        // describes has ended (or hit the window below).
        uint32_t flush_count;
        // Boot-relative ms of the first suppressed line in that run.
        uint32_t flush_first_ms;
    };

    // A run that never breaks would otherwise never be reported. Flushing on
    // this window turns "stuck repeating" from invisible into a line per
    // interval, which is the whole difference between a wedged machine that
    // looks silent and one that says so.
    static constexpr uint32_t flush_window_ms = 1000;

    Decision consider(const char* line, size_t len, uint32_t now_ms)
    {
        const size_t n = (len > line_max) ? line_max : len;
        const bool same = (n == last_len_) && (n != 0) && (memcmp(line, last_, n) == 0);

        if (!same)
        {
            const Decision d{true, run_, first_ms_};
            memcpy(last_, line, n);
            last_len_ = n;
            run_ = 0;
            return d;
        }

        // Identical. Start or extend the run.
        if (run_ == 0)
        {
            first_ms_ = now_ms;
        }
        run_++;

        if (now_ms - first_ms_ >= flush_window_ms)
        {
            const Decision d{false, run_, first_ms_};
            run_ = 0;
            // Deliberately NOT reset to now_ms: the next suppressed line starts
            // the next run and stamps it. Stamping here would date the run from
            // a line that was never suppressed.
            return d;
        }

        return Decision{false, 0, 0};
    }

    uint32_t suppressed() const
    {
        return suppressed_total_;
    }

    void countSuppressed(uint32_t n)
    {
        suppressed_total_ += n;
    }

  private:
    char last_[line_max] = {};
    size_t last_len_ = 0;
    uint32_t run_ = 0;
    uint32_t first_ms_ = 0;
    uint32_t suppressed_total_ = 0;
};

// --------------------------------------------------------------------------
// A4 -- per-category token bucket
// --------------------------------------------------------------------------

// Configured by **milliseconds per token**, not by a rate.
//
// A rate would need a division whose remainder has to be carried somewhere, and
// carrying it wrong is a slow drift that only shows up as a limiter that is
// tighter or looser than the comment claims. ms-per-token makes the sustained
// rate exact by construction: 1000 / ms_per_token lines per second, with
// `capacity` available as a burst.
class TokenBucket
{
  public:
    constexpr TokenBucket(uint32_t capacity, uint32_t ms_per_token)
        : capacity_(capacity), ms_per_token_(ms_per_token), tokens_(capacity)
    {
    }

    // True if a token was available. False means this line is dropped and
    // counted -- never blocked, never queued (SAF-50's rule, applied to the
    // limiter as well as to the ring).
    bool take(uint32_t now_ms)
    {
        refill(now_ms);
        if (tokens_ == 0)
        {
            denied_++;
            return false;
        }
        tokens_--;
        return true;
    }

    uint32_t denied() const
    {
        return denied_;
    }

    uint32_t tokens() const
    {
        return tokens_;
    }

  private:
    void refill(uint32_t now_ms)
    {
        if (!started_)
        {
            started_ = true;
            last_ms_ = now_ms;
            return;
        }
        // Unsigned wrap is correct here: time_us_64()/1000 is 32-bit truncated
        // by the caller and the difference stays right across the boundary.
        credit_ms_ += now_ms - last_ms_;
        last_ms_ = now_ms;

        while (credit_ms_ >= ms_per_token_ && tokens_ < capacity_)
        {
            tokens_++;
            credit_ms_ -= ms_per_token_;
        }
        if (tokens_ >= capacity_)
        {
            // Full. Holding credit here would let an idle category bank an
            // unbounded burst, which is the bound this class exists to provide.
            credit_ms_ = 0;
        }
    }

    uint32_t capacity_;
    uint32_t ms_per_token_;
    uint32_t tokens_;
    uint32_t credit_ms_ = 0;
    uint32_t last_ms_ = 0;
    uint32_t denied_ = 0;
    bool started_ = false;
};

// --------------------------------------------------------------------------
// Severity floors
// --------------------------------------------------------------------------

// Per-category minimum severity, with an optional expiry.
//
// **`enabled()` does NOT evaluate expiry.** IF-0002 section 10.2: the TTL is
// swept by the telemetry task through expire(), never on the emit path and
// never in link RX. Sweeping in link RX would mean a link drop strands the
// machine at TRACE with no way to reach it -- the one state this must not be
// able to enter. The cost is that a TTL overruns by up to one sweep interval,
// which is a second, and which nobody can observe.
class LevelTable
{
  public:
    explicit LevelTable(Level build_default)
    {
        for (size_t i = 0; i < cat_count; i++)
        {
            floor_[i] = build_default;
            expiry_ms_[i] = 0;
        }
        build_default_ = build_default;
    }

    bool enabled(Cat cat, Level level) const
    {
        return static_cast<uint8_t>(level) >= static_cast<uint8_t>(floor_[index(cat)]);
    }

    Level floor(Cat cat) const
    {
        return floor_[index(cat)];
    }

    Level buildDefault() const
    {
        return build_default_;
    }

    // ttl_s == 0 means "until reset or until changed again". Nothing here
    // survives a reset, by design: a persisted debug level is a permanent
    // degradation nobody remembers enabling (IF-0002 section 10.1).
    void setFloor(Cat cat, Level level, uint32_t ttl_s, uint32_t now_ms)
    {
        const size_t i = index(cat);
        floor_[i] = level;
        expiry_ms_[i] = (ttl_s == 0) ? 0 : now_ms + (ttl_s * 1000u);
    }

    void setAll(Level level, uint32_t ttl_s, uint32_t now_ms)
    {
        for (size_t i = 0; i < cat_count; i++)
        {
            setFloor(static_cast<Cat>(i), level, ttl_s, now_ms);
        }
    }

    // Reverts every expired category to the build default. Called from the
    // telemetry task; see the class comment for why not from anywhere else.
    // Returns how many categories reverted, so the caller can log the event
    // exactly once rather than on every sweep.
    uint32_t expire(uint32_t now_ms)
    {
        uint32_t reverted = 0;
        for (size_t i = 0; i < cat_count; i++)
        {
            if (expiry_ms_[i] == 0)
            {
                continue;
            }
            // Signed difference so the comparison survives the 32-bit ms wrap
            // at ~49.7 days of uptime. A machine that has been up that long and
            // reverts a TTL a sweep early is a better outcome than one that
            // never reverts it again.
            if (static_cast<int32_t>(now_ms - expiry_ms_[i]) >= 0)
            {
                floor_[i] = build_default_;
                expiry_ms_[i] = 0;
                reverted++;
            }
        }
        return reverted;
    }

  private:
    static size_t index(Cat cat)
    {
        const size_t i = static_cast<size_t>(cat);
        return (i < cat_count) ? i : static_cast<size_t>(Cat::other);
    }

    Level floor_[cat_count];
    uint32_t expiry_ms_[cat_count];
    Level build_default_ = Level::info;
};

} // namespace log_policy
