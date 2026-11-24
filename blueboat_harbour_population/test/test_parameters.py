"""The evidence file: its form, its vocabulary, the level tables."""
from blueboat_harbour_population import default_parameters
from blueboat_harbour_population.data import LEVELS, TAGS, TARGETS, code_digest, evidence_digest
from blueboat_harbour_population.sources import centred_affinity, kernel_cumulative, level_density

import numpy as np


def test_form():
    p = default_parameters()
    assert p.check() == []
    n = 0
    for path, rec in p.records():
        n += 1
        assert rec["tag"] in TAGS, path
        assert rec.get("used_by"), path
    assert n > 150


def test_levels_rise_and_meet_their_anchors():
    p = default_parameters()
    lam = [level_density(p, s) for s in LEVELS]
    assert all(b > a for a, b in zip(lam, lam[1:]))
    assert lam[0] == 1.0 and lam[4] == 10.0 and lam[9] == 100.0      # the three anchors of the scale
    # geometric steps: a decade from 1 to 5 and a decade from 5 to 10
    for s in range(1, 5):
        assert abs(lam[s] / lam[s - 1] - 10 ** 0.25) < 0.01
    for s in range(5, 10):
        assert abs(lam[s] / lam[s - 1] - 10 ** 0.2) < 0.01
    sl = p("levels.small_litter_per_m2")
    assert all(sl[b] > sl[a] for a, b in zip(LEVELS, LEVELS[1:]))
    assert sl[1] == 0.01 and sl[5] == 0.06 and sl[10] == 0.5       # the frozen R2 presets low, nominal, high
    g = p("levels.group_chance")
    assert all(g[b] >= g[a] for a, b in zip(LEVELS, LEVELS[1:])) and g[10] <= 0.5


def test_centred_affinity_reproduces_the_profile_weights():
    """A harbour with the profile's reference composition of sources has the
    profile's class weights."""
    p = default_parameters()
    for prof in p.profiles:
        ref = p(f"profiles.{prof}.reference")
        if not ref:
            continue
        W = p(f"profiles.{prof}.weights")
        tot = sum(W.values())
        g = centred_affinity(p, prof)
        got = {c: 0.0 for c in TARGETS}
        rs = sum(ref.values())
        for tag, r in ref.items():
            e = p(f"tags.{tag}.emits")
            norm = sum(W[c] * g[c] * e[c] for c in TARGETS)
            for c in TARGETS:
                got[c] += (r / rs) * W[c] * g[c] * e[c] / norm
        for c in TARGETS:
            assert abs(got[c] - W[c] / tot) < 2e-3, (prof, c, got[c], W[c] / tot)


def test_kernels_are_proper():
    p = default_parameters()
    d = np.linspace(0.0, 50.0, 501)
    for tag in p.tags:
        cum = kernel_cumulative(p, tag, d)
        assert abs(cum[-1] - 1.0) < 1e-9 and np.all(np.diff(cum) >= 0.0), tag


def test_digests():
    assert len(code_digest()) == 16 and len(evidence_digest()) == 16
    assert code_digest() == code_digest()
