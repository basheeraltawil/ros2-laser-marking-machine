Voice description (paste into the voice/style field)

A calm, confident automation engineer in his late 30s, speaking clear international English with a slight warmth, like a senior engineer giving a short tour of his own machine to visitors at a trade fair. Moderate pace, never rushed, with short pauses after each key idea so viewers can watch the visuals. Curious and slightly playful when he asks a question, steady and serious when he talks about safety and faults, quietly proud (never boastful) when he gives results and numbers. Crisp consonants on technical terms. No hype, no shouting, no sales voice.


Script (paste into the script field)

[curious, inviting] What does it take to turn an old machine into a smart one?
[calm, storytelling] This is a belt marking machine. It feeds a belt, a laser writes a label on it, and a pneumatic knife cuts it into pieces.
[reflective] The first version, built in twenty-nineteen, worked... but it was one long Arduino program. It could not stop safely, it could not report a fault, and it never knew when the laser had actually finished.
[confident, upbeat] So I rebuilt it from the ground up.
[clear, explaining] Today, a Raspberry Pi runs ROS two Humble, with a touchscreen for the operator. An Arduino Mega handles the real-time inputs and outputs, and talks to the Pi over a checked serial protocol with a watchdog.
[clear, steady] The laser keeps its own software. My system simply presses its foot pedal, through an isolated relay contact, and waits for the laser's done signal.
[rhythmic, slower] Feed the belt... mark the label... cut... count... next.
[intrigued] And here is the interesting part: everything you see runs in a Gazebo digital twin.
[explaining] The same control software drives the simulation and the real machine. Only the hardware layer changes.
[serious, steady] But real factories are not perfect. So I tested the faults.
[suspenseful] The belt runs out in the middle of a batch...
[calm, reassuring] Alarm E four-oh-one. The machine holds safely, the operator splices a new belt, presses resume... and not a single label is lost.
[firm, serious] An emergency stop? That is wired in hardware, not left to software. The software only reports it and guides the restart.
[proud, measured] Twelve industrial scenarios are tested automatically on every update. An eight-hour shift runs in about twenty-five seconds, with an overall equipment effectiveness of ninety-three point six percent.
[curious, lighter] And what about artificial intelligence?
[explaining] A camera checks every mark. A YOLO model finds weak marks and burn spots, and a measured contrast makes the final decision.
[explaining, thoughtful] Another model watches the knife. When it becomes twenty percent slower, the system warns maintenance after about sixteen cycles... long before it fails.
[firm] And one rule never changes: AI can advise, but it never controls safety.
[warm, steady] The engineering math, the wiring guide, the commissioning checklist and the code are all documented, step by step.
[warm, inviting] The whole project is open source. Search for ROS two laser marking machine on GitHub.
[friendly, final] If you build machines, or you want to learn how, subscribe... and let's build smarter machines together.
