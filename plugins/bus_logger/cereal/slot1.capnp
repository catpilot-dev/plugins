  # PluginBusLog — claims CustomReserved1
  entries @0 :List(Entry);

  struct Entry {
    topic @0 :Text;
    json @1 :Text;
    monoTime @2 :UInt64;   # monotonic nanoseconds
  }
