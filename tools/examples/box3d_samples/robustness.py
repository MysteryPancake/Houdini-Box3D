"""Box3D's Robustness samples (samples/sample_robustness.cpp, shared/overflow_color.c)."""

from .common import Bodies, finish, sample

SOURCE = "sample_robustness.cpp"


@sample("Robustness", "HighMassRatio1", SOURCE, frames=600)
def high_mass_ratio_1(s):
    b = Bodies(s.tag)
    b.ground(50.0)
    b.stream("boxes", b.box("boxes", 1.0, 1.0, 1.0), """\
// Three pyramids of ten light boxes (density 1), each topped by a heavy box: density 100, 200, 300.
float extent = 1.0;
for (int j = 0; j < 3; ++j) {
    int count = 10;
    float offset = -20.0 * extent + 2.0 * (count + 1.0) * extent * j;
    float y = extent;
    while (count > 0) {
        for (int i = 0; i < count; ++i) {
            float coeff = i - 0.5 * count;
            float yy = count == 1 ? y + 2.0 : y;
            int pt = body(set(2.0 * coeff * extent + offset, yy, 0.0), identity);
            float density = count == 1 ? (j + 1.0) * 100.0 : 1.0;
            setpointattrib(0, "density", pt, density);
            if (count == 1) setpointattrib(0, "Cd", pt, set(0.8, 0.25, 0.2));
        }
        --count;
        y += 2.0 * extent;
    }
}
""", cd=(0.9, 0.75, 0.45))
    return finish(s, b, about="""\
Three pyramids of light boxes, each with a box 100, 200 and 300 times as dense dropped on top. A solver
that handles mass ratios badly lets the heavy box sink through.""")


@sample("Robustness", "Tiny Pyramid", SOURCE, frames=600)
def tiny_pyramid(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.stream("boxes", b.box("boxes", 0.025, 0.025, 0.025), """\
// A pyramid 30 boxes wide of 5 cm boxes.
float e = 0.025;
int baseCount = 30;
for (int i = 0; i < baseCount; ++i) {
    float y = (2.0 * i + 1.0) * e;
    for (int j = i; j < baseCount; ++j) {
        float x = (i + 1.0) * e + 2.0 * (j - i) * e - baseCount * e;
        body(set(x, y, 0.0), identity);
    }
}
""", cd=(0.9, 0.6, 0.2))
    return finish(s, b, about="A pyramid of 465 boxes 5 cm across. Small bodies are harder to keep stacked; it "
                              "should stand.")


@sample("Robustness", "Overlap Recovery", SOURCE, frames=300)
def overlap_recovery(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.stream("boxes", b.box("boxes", 0.5, 0.5, 0.5), """\
// A pyramid four wide whose boxes overlap by a quarter of their size: the solver pushes them apart.
int baseCount = 4;
float overlap = 0.25, extent = 0.5;
float fraction = 1.0 - overlap;
float y = extent;
for (int i = 0; i < baseCount; ++i) {
    float x = fraction * extent * (i - baseCount);
    for (int j = i; j < baseCount; ++j) {
        body(set(x, y, 0.0), identity);
        x += 2.0 * fraction * extent;
    }
    y += 2.0 * fraction * extent;
}
""", cd=(0.9, 0.6, 0.2), density=1.0)
    return finish(s, b, about="""\
Ten boxes that start overlapping by a quarter of their size. Box3D pushes them apart no faster than the
Overlap Push Speed (3 m/s), so they separate without flying off.""", try_="""\
on the Box3D Solver, raise Overlap Push Speed or Contact Stiffness (the sample's Speed and Hertz) and
watch the pyramid jump.""")


@sample("Robustness", "Overflow Color Pile", SOURCE, frames=300)
def overflow_color_pile(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.stream("hub", b.box("hub", 0.5, 2.5, 0.5), "body({0, 2.5, 0}, identity);  // tall and heavy (density 50)",
             cd=(0.6, 0.6, 0.65), density=50.0)
    b.stream("neighbors", b.box("neighbors", 0.2, 0.2, 0.2), """\
// Five rings of five small boxes around the hub, each just touching it, alternate rings turned half a slot.
float half = 0.2;
float ringRadius = 0.5 + half - 0.03;
for (int ring = 0; ring < 5; ++ring) {
    float y = half + 0.05 + 0.5 * ring;
    float thetaOffset = (ring & 1) ? PI / 5.0 : 0.0;
    for (int slot = 0; slot < 5; ++slot) {
        float theta = thetaOffset + 2.0 * PI * slot / 5.0;
        int pt = body(set(ringRadius * cos(theta), y, ringRadius * sin(theta)), identity);
        setpointattrib(0, "Cd", pt, hsvtorgb(set(ring / 5.0, 0.6, 0.95)));
    }
}
""")
    return finish(s, b, about="""\
A heavy post with 25 small boxes around it, all touching it from the first step. So many contacts on one
body overflow Box3D's graph colours, the path this sample tests; the pile should settle calmly.""",
                  ported="the sample's overflow contact count is not shown.")
