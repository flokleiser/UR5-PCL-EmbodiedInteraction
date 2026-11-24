# First real test over Ethernet: steps

Short version of `real-robot-checklist.md` for today's test. Lab cell `cart_360`: the arm stays on its
cart and may work all around it.

## 0. Before the lab (on your Mac)

1. **Get the framework update.** Your copy has the `cart_360` docs, but not the code yet. Without it the
   real arm still uses the old table-only profile, which refuses every drawing pose. From the repo root:

   ```bash
   git pull https://github.com/reckj/UR5-PCL-EmbodiedInteraction.git main
   ```

   This brings the course commit "cart_360: framework code, cell file and viewer scene". It only
   touches `src/`, `scenes/`, `context/`, `docs/` and `examples/`, not `students/flo/`.

2. **Rehearse in the simulator against the cart**, so the simulator rejects exactly what the real arm
   would reject:

   ```bash
   ./start --cell cart_360
   ```

   On http://localhost:8765/ run **Testmuster**, then **Rundentest**. Both must finish without a red
   message. If a move is rejected (e.g. `the forearm would come closer than 60 mm …`), stop here and
   bring me the message.

3. **Weigh the lamp with its mount.** It must be **0.5 kg or less**, otherwise the framework refuses
   it.

4. **Measure the lamp TCP** if you can: the distance from the flange face to the lamp's centre line is
   still assumed to be 5 cm (`lamp_tcp` in `tonspur/config.json`).

## 1. At the robot

5. **Pendant: check the safety installation.** It must be `cart_360`, and the safety checksum (four
   characters, top right) must match the one posted at the robot. If not, stop and ask the instructor.
6. **Pendant: set the tool.** Under *Installation → General → TCP* enter
   `X 130.4 mm, Y -34.9 mm, Z 50 mm, RX 0.2055, RY 1.5610, RZ -0.2055` (rotation vector in rad, the same as `lamp_tcp`: lamp mounted sideways, turned 15° about the flange axis, as observed on the real arm) and the measured payload.
7. **Freedrive the arm** roughly upright, the elbow up and clear of the cart top. If any part of the
   arm is inside the cart's keep-out, the framework refuses every move ("the arm is ALREADY there").
8. **No cloth yet,** nobody inside the arm's reach, cables loose enough for one full turn of the base.
9. **Pendant:** robot ON → START, then switch the header to **Remote Control**. Set the speed slider
   to about 10–20 %.

## 2. Network

10. Connect the Ethernet cable from the robot to the Mac.
11. On the Mac, give the Ethernet port a manual IP: *System Settings → Network → Ethernet → Details →
    TCP/IP*, *Configure IPv4: Manually*, IP `192.168.1.100`, subnet mask `255.255.255.0`, no router.
12. Test it in the Mac terminal:

    ```bash
    ping -c 3 192.168.1.11
    nc -vz -w 3 192.168.1.11 30004
    ```

    Both must answer. If not: check the cable, and the robot's IP on the pendant (*menu → About*).

## 3. Start and connect

13. Start the server for the real robot, **without the simulator**:

    ```bash
    ./start --real
    ```

    This starts neither URSim nor the viewer. The page opens on **Echter Roboter** with `192.168.1.11`
    filled in, and the server waits: it doesn't connect to the arm until you press the button in step 16.
    (Different IP: `./start --real --host 192.168.1.x`.)

14. Open http://localhost:8765/ in **Chrome**.
15. **DMX.** Use the real DMX box, but cover the lamp, or leave the lamp unplugged from the power
    supply.
    1. Under *Lampe · DMX*, choose the interface type and press **DMX verbinden**.
    2. Pick the device in Chrome's list.
    3. Press **1 s testen**: the power supply should switch on for one second.

    Jobs on the real robot don't start without a connected DMX box.
16. **Connect the robot.** Under *Verbindung*, choose **Echter Roboter**, enter `192.168.1.11` and
    the lamp weight in kg, then press **Mit echtem Roboter verbinden** and confirm.
17. **Check** before going on:
    - the terminal shows `Verbindung gewechselt zu ECHTER ROBOTER 192.168.1.11` and
      `[ur5e] REAL ARM 192.168.1.11 — profile 'cart_360'`;
    - the badge at the top of the page is **red**;
    - *Verbindung* in the status list says "verbunden".

## 4. Test runs

Keep a hand on the red emergency stop at all times. The page's NOT-AUS button and Esc also stop the
arm and switch the lamp off, but they don't replace the red button.

17b. **Optional: movement test without DMX.** Under *Lampe · DMX*, tick **Trockenlauf**. Jobs then run
    without a DMX box and the lamp is never switched on (DMX gets "off" even if connected). The status
    list shows "TROCKENLAUF". Untick it before the exposure test (step 20). It switches itself off on every
    new connection.
18. **Testmuster, lamp covered.** Choose *Testmuster* under *Darstellung* and press **Muster an
    Roboter senden**.
    - The first job begins with the takeover: the arm pulls back, straightens up (elbow up) and moves
      into its checked posture beside the cloth radius.
    - Watch the whole job. On the page, the lamp should switch on only while a line or dot is drawn.
19. **Rundentest:** the arm goes once around the cloth at drawing distance, halfway up the band,
    with the lamp off, then the base turns back to 0°. Hang the cloth (or hold a tape measure) and
    check the lamp stays about 5 cm from it all the way round. Watch the cables at the cart's
    corners and the handle.
    Then **Höhentest:** at 0° the lamp goes to the highest drawable point (0.60 m above the base),
    then to the lowest (0.05 m), waiting 3 s at each. Check the lamp, wrist and cables clear the
    cloth's top and hem and the cart.
20. **Exposure test.** Pin a scrap of cyanotype fabric about 5 cm in front of the lamp's path,
    uncover the lamp, and run the Testmuster again. Its five lines (20–150 mm/s) and five dots
    (0.25–4 s) show which speed gives which darkness.
21. **Slider up step by step,** only after everything above went smoothly.

## 5. Finish

22. On the page, under *Verbindung*, choose **Simulator** and press **Verbinden**. Or stop the server
    with Ctrl+C: that triggers NOT-AUS.
23. Press **DMX trennen**, then switch the lamp's power supply off.
24. Pendant: switch to Local mode, then power the robot off as usual.

## If something goes wrong

| what you see | what it means | what to do |
|---|---|---|
| page badge stays green / "Simulator" | still connected to the simulator | redo step 16 |
| "antwortet nicht auf Port 30004" under *Verbindung* | the Mac doesn't reach the robot | step 10–12: cable, manual IP on the Ethernet port, `ping`; the page keeps working meanwhile |
| "Echter Roboter gesperrt", button greyed out | server started with plain `./start` | restart with `./start --real` |
| "Verbindung: …" error under *Verbindung* after pressing the button | the server (in Docker) can't open RTDE to the arm | check the pendant is in Remote Control; in the terminal `nc -vz -w 3 192.168.1.11 30004` |
| `profile 'cart_against_wall'` in the terminal | framework update missing | step 1, then restart |
| `profile 'cart_360' is not validated` | cell not released for the real arm | ask the instructor |
| `payload not set` / payload above 0.5 kg | lamp weight missing or too heavy | enter the real weight, max 0.5 kg |
| `the arm is ALREADY there` | arm stands inside the cart's keep-out | freedrive it clear, NOT-AUS → Zurücksetzen |
| `… would come closer than 60 mm …` / `passes through the keep-out` | a move would touch the cart | nothing was sent; note the message and bring it to me |
| "wartet auf DMX" | no DMX box connected on the page | step 15 |
| "DMX hat … nicht bestätigt" → NOT-AUS | DMX box or page didn't answer within 1 s | check the USB cable, reconnect DMX, Zurücksetzen |
| `RTDE control script is not running` | robot not in Remote Control, or protective stop | pendant: clear the stop, Remote Control, then Zurücksetzen |
| protective stop on the pendant | the controller stopped the arm | write down the code and text before acknowledging |
