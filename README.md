# Mind the Gap: SUMO lane-merge demo

This repository currently contains the SUMO/TraCI side of the RSA project.
The road network is a minimal one-lane main road plus one on-ramp that joins at
`merge_point`.

## Run

Small synthetic map, one scenario at a time:

```bash
python3 scripts/run_traci.py --map merge --scenario base --gui
python3 scripts/run_traci.py --map merge --scenario gap --gui
python3 scripts/run_traci.py --map merge --scenario adaptive --gui
python3 scripts/run_traci.py --map merge --scenario loss --gui --end 80
```

Small synthetic map, all scenarios:

```bash
python3 scripts/run_traci.py --map merge --all --gui --end 80
```

Aveiro OSM map:

```bash
sumo-gui -c aveiro_map/aveiro.sumocfg
```

Aveiro V2X merge scenarios, one at a time:

```bash
python3 scripts/run_traci.py --map aveiro --scenario base --gui
python3 scripts/run_traci.py --map aveiro --scenario gap --gui
python3 scripts/run_traci.py --map aveiro --scenario adaptive --gui
python3 scripts/run_traci.py --map aveiro --scenario loss --gui --end 80
```

Slower presentation mode:

```bash
python3 scripts/run_traci.py --map aveiro --scenario adaptive --demo
python3 scripts/run_traci.py --map aveiro --scenario loss --demo --end 80
```

Manual speed control for SUMO-GUI playback:

```bash
python3 scripts/run_traci.py --map aveiro --scenario adaptive --gui --delay 250
```

The GUI run follows the ramp vehicle, draws a smaller colored marker around
each vehicle and adds a real-time status badge next to it:

- gray: driving normally
- yellow: merge request / negotiation
- green: accepted or released
- cyan: cooperative yielding
- red: blocked or no fresh CAMs
- purple: merged

If the vehicles are still too small, increase the marker size or zoom:

```bash
python3 scripts/run_traci.py --map aveiro --scenario adaptive --gui --marker-radius 12 --badge-size 6 --zoom 1400
```



Each CAM line contains:

- `itsPduHeader`: `protocolVersion`, `messageID=2`, `stationID`
- `cam.generationDeltaTime`
- `basicContainer.referencePosition`: latitude/longitude
- `basicVehicleContainerHighFrequency`: heading, speed, acceleration, vehicle size
- `sumo`: vehicle id, role, road, lane, lane position and x/y position
- `mergeApplication`: status, distance/ETA to merge, reservation and gate state
- `channel`: communication range and delivered receivers

To choose explicit output files:

```bash
python3 scripts/run_traci.py --map aveiro --scenario adaptive --gui \
  --log-file logs/demo.log \
  --cam-log-file logs/demo.cam.jsonl
```

Repeat a scenario a fixed number of times in the same SUMO run:

```bash
python3 scripts/run_traci.py --map aveiro --scenario adaptive --gui --repeat 5 --end 400
```

Loop a scenario forever. Vehicles leave the merge area and a new cycle is
spawned from the start:

```bash
python3 scripts/run_traci.py --map aveiro --scenario adaptive --gui --loop
```

Loop all scenarios forever in sequence:

```bash
python3 scripts/run_traci.py --map aveiro --all --gui --loop --end 80
```

If TraCI cannot be imported, set `SUMO_HOME` to the SUMO install directory. On
Ubuntu packages this is usually not needed because the script also checks
`/usr/share/sumo/tools`.

The Aveiro map was converted from `aveiro_map/aveiro.osm` with `netconvert` and
includes `aveiro_map/aveiro.net.xml` plus `aveiro_map/aveiro.poly.xml` for GUI
context.

The conversion used SUMO's OSM typemap, passenger-road filtering, junction
joining, roundabout/ramp guessing and OSM turn-lane import.

## Scenarios

- `base`: two vehicles reach the merge with a conflict; the ramp vehicle yields
  and merges after the main-lane vehicle.
- `gap`: two main-lane vehicles create a safe temporal gap; the ramp vehicle
  reserves the gap and merges between them.
- `adaptive`: the gap is initially too small; the following main-lane vehicle
  cooperates by slowing down.
- `loss`: V2X messages involving the ramp vehicle are dropped during
  negotiation; the ramp vehicle falls back to the safe behaviour and waits.

## Current algorithm

Vehicles periodically exchange CAM-like state messages in the TraCI process:
position, speed, acceleration, road/lane and estimated time of arrival at the
merge point. Message delivery is filtered by communication range and, in the
`loss` scenario, by a forced outage window.

When the ramp vehicle enters the request zone it sends a merge request. The
controller checks the temporal gap at the merge point using a minimum gap of
2.5 s. If the current ETA conflicts with an earlier main-lane ETA, the ramp
vehicle shifts its target ETA and slows down. If the ramp vehicle has priority
and the scenario allows cooperation, a following main-lane vehicle receives a
yield target and slows to open the gap.

The safe fallback is conservative: if there are no fresh CAMs or no accepted
reservation, the ramp vehicle slows down and stops before the merge point.

SUMO does not decide whether the ramp vehicle may merge. The route still
exists in the map because SUMO needs the physical road connection, but the ramp
vehicle starts with a hold route that ends before the merge. The TraCI
controller enforces a manual gate before the merge point. The ramp vehicle only
receives the full route after the Python controller has accepted a reservation
and emitted `MANUAL_MERGE_RELEASE` / `ROUTE_GRANTED`; otherwise it remains
blocked at the gate.

## Metrics printed

- ramp merge success
- collision count
- minimum observed vehicle distance
- minimum actual time gap between vehicles entering `main_out`
- average negotiation time
- number of CAM/request/accept/cooperation/timeout events
- number of manual gate holds/releases
- unauthorized ramp merges
- minimum speed per vehicle
