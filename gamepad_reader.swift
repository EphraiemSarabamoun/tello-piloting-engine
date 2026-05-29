// Reads an Xbox/standard controller via Apple's GameController framework and
// streams its state as one JSON object per line (~50 Hz) to stdout.
//
// GameController is exempt from the Input Monitoring privacy gate that blocks
// SDL's raw-HID path on macOS. But GC only refreshes input *values* when there
// is an active NSApplication run loop bound to the window server -- a bare
// `RunLoop.main.run()` CLI connects the pad yet reads flat zero. So this runs as
// a windowless, dockless .accessory NSApplication.
//
// GC axis convention: thumbstick up = +1, right = +1 (already Mode-2 friendly).
//
//   swiftc -O gamepad_reader.swift -o gamepad-reader -framework GameController -framework AppKit
//   ./gamepad-reader            # JSON lines; Ctrl-C to stop

import AppKit
import Foundation
import GameController

func emit(_ s: String) {
    print(s)
    fflush(stdout)
}

final class Reader {
    var pad: GCExtendedGamepad?

    func bind(_ c: GCController) {
        pad = c.extendedGamepad
        emit("{\"event\":\"connect\",\"vendor\":\"\(c.vendorName ?? "?")\"}")
    }

    func start() {
        // Deliver input even when this process isn't the frontmost app.
        GCController.shouldMonitorBackgroundEvents = true

        NotificationCenter.default.addObserver(
            forName: .GCControllerDidConnect, object: nil, queue: .main
        ) { [weak self] note in
            if let c = note.object as? GCController { self?.bind(c) }
        }
        NotificationCenter.default.addObserver(
            forName: .GCControllerDidDisconnect, object: nil, queue: .main
        ) { [weak self] _ in
            self?.pad = nil
            emit("{\"event\":\"disconnect\"}")
        }

        for c in GCController.controllers() { bind(c) }
        GCController.startWirelessControllerDiscovery(completionHandler: {})

        let timer = Timer(timeInterval: 0.02, repeats: true) { [weak self] _ in
            func b(_ x: GCControllerButtonInput?) -> Int { (x?.isPressed ?? false) ? 1 : 0 }
            guard let g = self?.pad else {
                emit("{\"connected\":false}")
                return
            }
            let lx = g.leftThumbstick.xAxis.value
            let ly = g.leftThumbstick.yAxis.value
            let rx = g.rightThumbstick.xAxis.value
            let ry = g.rightThumbstick.yAxis.value
            let line = "{\"connected\":true,\"lx\":\(lx),\"ly\":\(ly),\"rx\":\(rx),\"ry\":\(ry),"
                + "\"a\":\(b(g.buttonA)),\"b\":\(b(g.buttonB)),\"x\":\(b(g.buttonX)),\"y\":\(b(g.buttonY)),"
                + "\"lb\":\(b(g.leftShoulder)),\"rb\":\(b(g.rightShoulder)),"
                + "\"start\":\(b(g.buttonMenu)),\"back\":\(b(g.buttonOptions)),"
                + "\"lt\":\(g.leftTrigger.value),\"rt\":\(g.rightTrigger.value)}"
            emit(line)
        }
        RunLoop.main.add(timer, forMode: .common)
    }
}

let app = NSApplication.shared
app.setActivationPolicy(.accessory)  // no dock icon, no window, but a real app run loop
let reader = Reader()
reader.start()
app.run()
