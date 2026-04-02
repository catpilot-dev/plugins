@0xb5e5f44e3ff0ea5a;

struct Coordinates {
  latitude @0 :Float64;
  longitude @1 :Float64;
}

struct Way {
  name @0 :Text;
  ref @1 :Text;
  maxSpeed @2 :Float64;
  advisorySpeed @3 :Float64;
  minLat @4 :Float64;
  minLon @5 :Float64;
  maxLat @6 :Float64;
  maxLon @7 :Float64;
  nodes @8 :List(Coordinates);
  lanes @9 :UInt8;
  hazard @10 :Text;
  oneWay @11 :Bool;
  maxSpeedForward @12 :Float64;
  maxSpeedBackward @13 :Float64;
}

struct Offline {
  minLat @0 :Float64;
  minLon @1 :Float64;
  maxLat @2 :Float64;
  maxLon @3 :Float64;
  ways @4 :List(Way);
  overlap @5 :Float64;
}
