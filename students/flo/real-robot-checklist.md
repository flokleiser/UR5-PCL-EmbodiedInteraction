# Tonspur-Tuch: from the simulator to the real UR5e

Checklist for moving the UV-lamp plotter (`students/flo/tonspur/`) from URSim to the real robot in the
lab cell **`cart_360`** (the lab setup since 2026-11-18, see `docs/SAFETY_SETUP.md` and
`context/CART_360.md`).

State: works in the simulator. Lamp side-mounted, rotated 180° around its beam (wrist above the lamp),
drawing band 0.05–0.60 m above the robot base, nothing below the base. The lamp is switched by a
DMX-controlled power supply (`"lamp_control": "dmx"`): the arm stays at drawing distance and the lamp
is only on while a line or dot is being drawn.

## What `cart_360` means for this project

- **The old blocker is gone.** The robot stays on its cart and may work all around it. `cart_360` is
  the framework's default profile for the real arm, so the server needs no special profile.
- **The cloth hangs around the cart.** It is centred on the robot's axis, not on the cart. The robot
  stands near the back of the cart: the cart reaches 0.566 m to the front (−X), 0.124 m to the back
  (+X), 0.345 m to the sides (±Y), plus the handle 60 mm further out on −Y. Its farthest corner is
  about 0.70 m from the robot's axis, so a cloth of radius 0.844 m clears it by about 15 cm.
- **The plotter's poses fit the cell.** Checked against the numbers in the docs:
  - all drawing moves are above the cart top;
  - the arm joints stay at least 0.20 m above the base, where the rule asks for 60 mm above the cart
    top, i.e. 0.05 m above the base;
  - the TCP stays inside the ±0.95 m reach box.
  The framework checks every move again before sending it.
- **Payload ceiling 0.5 kg.** The framework refuses a heavier tool. The lamp with its mount has to
  weigh 0.5 kg or less.
- **The pendant has no safety planes** and must not be changed. The installation `cart_360` is set up
  by the instructor and identified by its safety checksum (posted at the robot). The base plane in
  the plotter's 3D-viewer scene is only a drawing; the software watchdog still stops the arm if
  anything goes below the base.
- **Ceilings:** TCP 0.15 m/s and 0.5 m/s², joints 0.5 rad/s. A full rewind (one turn of the base)
  takes about 13 s.

## A. Before the lab session

- [ ] **Update the framework.** This checkout has the `cart_360` docs, but not the framework code
      they describe: there is no `src/ur5e/cells/cart_360.json`, no `--cell` option and no viewer
      scene `cart_360`. Pull the course repository's latest `main` (`reckj/main`) before the lab.
      Then check:

  ```bash
  python -m ur5e.cell
  ```

  It prints the cart size, the clearances and the floor height the framework uses. Compare them
  with the numbers above.
- [ ] **Plan the cables.** The base turns one full lap and then rewinds. The lamp cable, the DMX
      power-supply cable and the robot cables must allow at least one full turn. Make sure they can't
      catch on the cart's corners or the handle. The checks don't see cables.
- [ ] **Plan the cloth frame.** The cloth has to hang centred on the robot's axis, about 30 cm behind
      the middle of the cart. Measure from the robot's base, not from the cart. The frame or stand
      holding it is not in the cell: keep it outside the cloth.

## B. Measure the real lamp and room

Enter the values in `students/flo/tonspur/config.json`.

- [ ] **Lamp TCP.** These numbers go into the pendant (*Installation → General → TCP*) **and** into
      `lamp_tcp`, and they must match exactly:
  - [ ] how far the lamp tip sticks out sideways from the flange axis (currently `0.135` m);
  - [ ] distance along the flange axis from the flange face to the lamp's centre line
        (currently `0.05` m, **assumed, not measured**);
  - [ ] which way the lamp points as seen on the pendant (currently flange +X:
        `"lamp_tcp": [0.135, 0.0, 0.05, 0.0, 1.5708, 0.0]`).
- [ ] **Lamp width.** The framework only checks the line from the flange to the lamp tip. The lamp
      body is 20 cm long and sticks out to the side; the plotter keeps it at least 3 cm above the
      base by itself.
- [ ] **Weigh the lamp with its mount** (kg, at most **0.5 kg**). You enter it on the page when you
      connect to the real robot.
- [ ] **DMX power supply:**
  - [ ] its DMX address → `dmx_channel` (currently 1);
  - [ ] which value switches it on → `dmx_on_value` (currently 255) and off → `dmx_off_value` (0);
  - [ ] set it to switch **off when the DMX signal is lost** (often called "signal loss: blackout"
        rather than "hold last value"), if it has that setting;
  - [ ] which USB interface it is: Enttec DMX USB Pro (or compatible), or an Open DMX / FTDI cable.
- [ ] **Cloth:**
  - [ ] real radius → `cloth_radius_m` (currently 0.844). It must stay above about 0.72 m to clear
        the cart's corners and handle;
  - [ ] height relative to the robot base → `cloth_bottom_m` (currently −0.425). The base sits about
        0.80 m above the floor on the cart (the cart is 790 mm tall plus a 10 mm plate), so −0.425
        puts the hem about 37.5 cm above the floor;
  - [ ] position of the seam → `start_angle_deg` (currently −180). Angles are in the robot's base
        frame: +X points to the back of the cart, −X to the front, where the handle side is −Y.
- [ ] **After any change to `config.json`, start the server once in the simulator.** It reruns the
      reach check and refuses to start if a pose is unreachable, too close to a singularity, or below
      the base.

## C. Rehearse in the simulator with the real values

- [ ] Enter the measured TCP in the simulated pendant as well.
- [ ] **Against the lab cell**, once the framework update from A is in: start the server with the
      cell option, so the simulator rejects exactly what the real arm would reject:

  ```bash
  ./start --cell cart_360
  ```

  If a move is rejected, the page shows the framework's message (e.g. `the forearm would come closer
  than 60 mm …`). Bring it to me; don't change the cell or the profile.
- [ ] Run **Testmuster** and **Rundentest**:
  - [ ] no rejections and no protective stops;
  - [ ] lap and rewind look right;
  - [ ] in the 3D viewer (http://localhost:7001/) the arm stays clear of the cloth and above the base
        plane.

## D. Connect to the real robot in the lab

- [ ] **Pendant:** check that the installation is `cart_360` and that the safety checksum (top right)
      matches the one posted at the robot. If not, stop and ask the instructor.
- [ ] **Network:** Ethernet cable in, Mac on a manual IP `192.168.1.1NN`, mask `255.255.255.0`.
      Test outside Docker:

  ```bash
  ping -c 3 192.168.1.11
  nc -vz -w 3 192.168.1.11 30004
  ```

- [ ] **Pendant:** robot ON → START, header switched to **Remote Control**, TCP and payload set,
      speed slider down to about 10–20 % for the first runs.
- [ ] **Start everything with the real robot unlocked** (from the repo root):

  ```bash
  ./start --allow-real
  ```

  Without `--allow-real`, the page only connects to the simulator.
- [ ] **DMX:** plug the USB interface into the Mac (not into Docker). On http://localhost:8765/
      choose the interface type under *Lampe · DMX*, press **DMX verbinden**, pick the device in
      Chrome's list, then **1 s testen**: the lamp must flash for one second. When connected to the
      real robot, the server won't start a job until the page reports a connected DMX interface.
- [ ] **Connect on the page.** Under *Verbindung*, choose **Echter Roboter**, enter the IP
      (`192.168.1.11`) and the measured lamp weight in kg, and press **Mit echtem Roboter verbinden**.
      Waiting jobs are cleared when you switch.
  - [ ] the terminal shows `Verbindung gewechselt zu ECHTER ROBOTER 192.168.1.11` and the
        framework's line `[ur5e] REAL ARM … profile 'cart_360'`;
  - [ ] the badge on the page is **red**.

  If it shows SIMULATOR, stop: it is connected to the wrong robot. If it says
  `profile 'cart_360' is not validated`, the cell hasn't been released for the real arm yet: ask the
  instructor.

  The old way still works: `docker compose run --rm --no-deps -e UR_HOST=192.168.1.11 -p 8765:8765 dev bash`,
  then `python students/flo/tonspur/server.py --target real --payload 0.4`.
- [ ] **3D viewer for the real robot (optional).** Stop the simulator viewer first:

  ```bash
  docker compose stop viewer
  docker compose run --rm --no-deps -e UR_HOST=192.168.1.11 -p 7001:7000 dev python scripts/rtde_viewer.py --target real --scene flo-tuch
  ```

  The scene `flo-tuch` shows the cloth but not the cart; `cart_360` shows the cart but not the cloth.

## E. First runs, step by step

- [ ] **DMX disconnected or lamp covered, no cloth.**
  - First freedrive the arm roughly upright and clear of the cart. If it stands inside the cart's
    keep-out, the framework refuses every move ("the arm is ALREADY there").
  - The server's first takeover moves the arm through a tucked, elbow-up pose into the checked
    posture.
  - Then send the **Testmuster** and watch the whole job with a hand on the red emergency stop. The
    page's NOT-AUS button and the Esc key also work, but they don't replace the red button.
- [ ] **Rundentest:** one full lap plus the rewind. Watch the cables at the cart's corners and the
      handle.
- [ ] **Exposure test** with the lamp on, on a scrap of cyanotype cloth: run the Testmuster. Its five
      lines (20–150 mm/s) and five dots (0.25–4 s) show which speed gives which darkness. Set
      `draw_speed_dark_mps`, `draw_speed_light_mps`, `point_dwell_s` and `dot_dwell_max_s` from the
      result, then restart the server.
- [ ] **Only then the real cloth and the microphones.** Raise the speed slider step by step.

## F. During operation

- **The lamp is switched by the web page.** Keep the page open in Chrome while drawing; it holds the
  connection to the DMX interface. NOT-AUS (button or Esc) switches the lamp off immediately. If the
  page is closed or the tab crashes while the lamp is on, the server stops the arm within about 2.5 s,
  but an Enttec Pro keeps sending the last value: then switch the power supply off by hand.
- **On the real arm the speed ceiling is 0.15 m/s** (joints 0.5 rad/s). Moves are slower than in the
  simulator, so jobs take longer than the page's estimate.
- **If a move is rejected** (`SafetyViolation`), nothing was sent and the arm hasn't moved. The job
  stops with the framework's message; note it down.
- **If there's a protective stop,** write down the pendant's code and text before acknowledging it.
- **Built-in guards:**
  - **pendant installation `cart_360`:** force 100 N, tool speed 250 mm/s. This layer counts if the
    arm touches something. It has no position limits;
  - **framework cell `cart_360`:** checks every move along its path against the cart and the floor;
  - **plotter's own software:**
    - the server checks every move path at startup;
    - a watchdog triggers the emergency stop if any joint or the lamp goes below the base;
    - the arm only starts a line after the page has confirmed "lamp on", and only moves on after it
      has confirmed "lamp off"; no confirmation within 1 s (`lamp_switch_timeout_s`) means emergency
      stop.
- **What no check sees:** the lamp body beside the flange–tip line, the cables, the cloth and its
  frame, and people.
