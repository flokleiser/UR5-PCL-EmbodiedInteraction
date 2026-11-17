# Tonspur-Tuch: from the simulator to the real UR5e

Checklist for moving the UV-lamp plotter (`students/flo/tonspur/`) from URSim to the real robot.
State: works in the simulator. Lamp side-mounted, rotated 180° around its beam (wrist above the lamp),
drawing band 0.05–0.60 m above the robot base, nothing below the base.

> **Blocker:** with the current lab setup the framework refuses every move. The real-arm safety profile
> `cart_against_wall` only allows a small box in front of the cart (at most about 0.56 m from the base);
> the drawing poses are 0.79 m out, all around the robot. Only the instructor may create or change
> real-arm profiles. Section A has to be done first.

## A. With the instructor, before the lab session

- [ ] **Set the robot up in the centre of the cloth.** On its stand, base about 78 cm above the floor
      (currently it stands on the cart against the wall).
- [ ] **Get a 360° safety profile.** The instructor creates and validates a real-arm profile for the
      centre setup: a box around the whole circle. The server then runs with that profile instead of
      `cart_against_wall`.
- [ ] **Update the robot's safety configuration on the real pendant** (*Installation → Safety*):
  - [ ] remove or adapt the wall and table planes (they don't fit a centre setup);
  - [ ] add the base plane: feature *Base*, displacement *0 mm*, mode *Normal*, *Restrict Elbow* on;
  - [ ] check the speed limits.
- [ ] **Plan the cables.** The base turns one full lap and then rewinds. The lamp cable and the robot
      cables must allow at least one full turn.

## B. Measure the real lamp and room

Enter the values in `students/flo/tonspur/config.json`.

- [ ] **Lamp TCP.** These numbers go into the pendant (*Installation → General → TCP*) **and** into
      `lamp_tcp`, and they must match exactly:
  - [ ] how far the lamp tip sticks out sideways from the flange axis (currently `0.135` m);
  - [ ] distance along the flange axis from the flange face to the lamp's centre line
        (currently `0.05` m, **assumed, not measured**);
  - [ ] which way the lamp points as seen on the pendant (currently flange +X:
        `"lamp_tcp": [0.135, 0.0, 0.05, 0.0, 1.5708, 0.0]`).
- [ ] **Weigh the lamp with its mount** (kg). Passed as `--payload` when starting the server.
- [ ] **Cloth:**
  - [ ] real radius → `cloth_radius_m` (currently 0.844);
  - [ ] height relative to the robot base → `cloth_bottom_m` (currently −0.425, i.e. the hem about
        35.5 cm above the floor with the base at 78 cm);
  - [ ] position of the seam → `start_angle_deg` (currently −180). Angles are in the robot's base
        frame: check on the pendant where the robot's +X axis points in the room.
- [ ] **After any change to `config.json`, start the server once in the simulator.** It reruns the
      reach check and refuses to start if a pose is unreachable, too close to a singularity, or below
      the base.

## C. Rehearse in the simulator with the real values

- [ ] Enter the measured TCP in the simulated pendant as well.
- [ ] Run **Testmuster** and **Rundentest**:
  - [ ] the base plane in the 3D viewer (http://localhost:7001/) stays yellow;
  - [ ] no protective stops;
  - [ ] lap and rewind look right.

## D. Connect to the real robot in the lab

- [ ] **Network:** Ethernet cable in, Mac on a manual IP `192.168.1.1NN`, mask `255.255.255.0`.
      Test outside Docker:

  ```bash
  ping -c 3 192.168.1.11
  nc -vz -w 3 192.168.1.11 30004
  ```

- [ ] **Pendant:** robot ON → START, header switched to **Remote Control**, TCP and payload set,
      speed slider down to about 10–20 % for the first runs.
- [ ] **Dev shell against the real robot**, with the page's port published:

  ```bash
  docker compose run --rm --no-deps -e UR_HOST=192.168.1.11 -p 8765:8765 dev bash
  ```

- [ ] **Start the server** with the measured weight in kg (it refuses to start without `--payload`):

  ```bash
  python students/flo/tonspur/server.py --target real --payload 1.2
  ```

  - [ ] terminal shows `ZIEL: !!! ECHTER ROBOTER 192.168.1.11 !!!`;
  - [ ] badge on http://localhost:8765/ is **red**.

  If it shows SIMULATOR, stop: it is connected to the wrong robot.
- [ ] **3D viewer for the real robot (optional).** Stop the simulator viewer first:

  ```bash
  docker compose stop viewer
  docker compose run --rm --no-deps -e UR_HOST=192.168.1.11 -p 7001:7000 dev python scripts/rtde_viewer.py --target real --scene flo-tuch
  ```

## E. First runs, step by step

- [ ] **Lamp off or covered, no cloth.** First raise the arm roughly upright with freedrive. The
      server's first takeover moves the arm through a tucked, elbow-up pose into the checked posture.
      Then send the **Testmuster** and watch the whole job with a hand on the red emergency stop.
      The page's NOT-AUS button and the Esc key also work, but they don't replace the red button.
- [ ] **Rundentest:** one full lap plus the rewind. Watch the cables.
- [ ] **Exposure test** with the lamp on, on a scrap of cyanotype cloth: run the Testmuster. Its five
      lines (20–150 mm/s) and five dots (0.25–4 s) show which speed gives which darkness. Set
      `draw_speed_dark_mps`, `draw_speed_light_mps`, `point_dwell_s` and `dot_dwell_max_s` from the
      result, then restart the server.
- [ ] **Only then the real cloth and the microphones.** Raise the speed slider step by step.

## F. During operation

- **The lamp can't be switched off from code.** After an emergency stop it keeps shining on one
  spot: cover it, or press "Zurücksetzen und Lampe wegdrehen".
- **On the real arm, the framework's speed ceiling is 0.15 m/s.** Travel moves are slower than in
  the simulator, so jobs take longer than the page's estimate.
- **If there's a protective stop,** write down the pendant's code and text before acknowledging it.
- **Built-in guards** (software, secondary to the pendant's safety configuration):
  - the server checks every move path at startup;
  - a watchdog triggers the emergency stop if any joint or the lamp goes below the base.
