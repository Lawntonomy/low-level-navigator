// Host-side unit tests for the BOOTSEL-reboot gate.
//
// The gate is the whole safety argument for a message that reboots the
// controller, and it is pure arithmetic over two inputs — exactly the shape
// that can be inverted, short-circuited, or made vacuous by a refactor without
// any bench session noticing. The hardware half (rom_reset_usb_boot, the drain
// ordering in link_tx_task) is not testable here and is not tested.

#include <gtest/gtest.h>

#include "app/bootloader.hpp"

using bootloader::judge;
using bootloader::Verdict;

// --------------------------------------------------------------------------
// The magic
// --------------------------------------------------------------------------

TEST(BootloaderGate, AcceptsTheMagicWhenDisarmed)
{
    EXPECT_EQ(judge(bootloader::request_magic, false), Verdict::accept);
}

TEST(BootloaderGate, RejectsZeroMagic)
{
    // The value a zeroed or truncated payload most plausibly produces.
    EXPECT_EQ(judge(0u, false), Verdict::bad_magic);
}

TEST(BootloaderGate, RejectsAllOnesMagic)
{
    EXPECT_EQ(judge(0xFFFFFFFFu, false), Verdict::bad_magic);
}

TEST(BootloaderGate, RejectsSingleBitCorruptionsOfTheMagic)
{
    for (int bit = 0; bit < 32; ++bit)
    {
        const uint32_t corrupted = bootloader::request_magic ^ (1u << bit);
        EXPECT_EQ(judge(corrupted, false), Verdict::bad_magic) << "bit " << bit;
    }
}

TEST(BootloaderGate, RejectsTheArmMagic)
{
    // safety.cpp's arm_magic, whole and zero-extended into the wider field.
    // Neither form may be mistaken for a bootloader request.
    EXPECT_EQ(judge(0xA57Eu, false), Verdict::bad_magic);
    EXPECT_EQ(judge(0xA57E0000u, false), Verdict::bad_magic);
}

TEST(BootloaderGate, MagicIsNotConfusableWithTheArmMagic)
{
    // Guards the choice itself, not the code: shrinking the bootloader magic to
    // 16 bits, or picking a value whose low half is the arm magic, would make
    // one message's payload a valid instance of the other's.
    EXPECT_NE(bootloader::request_magic & 0xFFFFu, 0xA57Eu);
    EXPECT_GT(bootloader::request_magic, 0xFFFFu);
}

// --------------------------------------------------------------------------
// The arm state
// --------------------------------------------------------------------------

TEST(BootloaderGate, RefusesWhileArmedEvenWithTheCorrectMagic)
{
    EXPECT_EQ(judge(bootloader::request_magic, true), Verdict::armed);
}

TEST(BootloaderGate, ArmedWithWrongMagicReportsTheMagic)
{
    // Ordering is deliberate: a frame that never asked for a reboot is a bad
    // frame, not an arming refusal, and reporting it as the latter would send
    // the Pi looking at its state machine instead of at its payload.
    EXPECT_EQ(judge(0xDEADBEEFu, true), Verdict::bad_magic);
}

TEST(BootloaderGate, NothingButAcceptEnablesTheReboot)
{
    // The caller branches on == accept, so this is the property that matters:
    // there is no third input combination that yields accept.
    EXPECT_NE(judge(0u, false), Verdict::accept);
    EXPECT_NE(judge(0u, true), Verdict::accept);
    EXPECT_NE(judge(bootloader::request_magic, true), Verdict::accept);
    EXPECT_EQ(judge(bootloader::request_magic, false), Verdict::accept);
}
