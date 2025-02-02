// we need to add states so we can successfully go from forware to backware without jerking the
// machine a machine should never go from commanding forward, to straight backward state transitions
// consist of: forward -> stopped stopped -> forward backward -> stopped stopped -> backward
