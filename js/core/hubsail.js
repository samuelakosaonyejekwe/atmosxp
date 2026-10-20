// Hub sail hardware (every design that uses the sail position): ONE bistable lifter ring per drive hub — a snap-through conical
// annular spring (Belleville geometry with h0/t > 2√2, so it has two stable states). The hub cable branch snaps it over; in its
// second state its inner lip holds the rocking pads off the cam track. Almen–László load–deflection, spring steel 17-7PH.
export function bistableLifter({ R = 0.24, E = 200e9, nu = 0.3, t = 0.0005, h0 = 0.0022, su = 1650e6 /* 17-7PH CH900 (conservative) */, nPad = 48, padArmK = 93, lift = 0.003, aVib = 30 * 9.81, nCycles = 2 * 365 * 15, kf = 60, kLev = 8, Fpeak = 150, dpre = 0.0004, camAmp = 0.0014, nCart = 24, diF = 0.35 /* annulus from the axle boss to the pad lip */, branchCap = Math.exp(0.10 * Math.PI), branchRatio = 12, etaLever = 0.9 } = {}) {
  const rc = R * 0.42, Do = 2 * rc * 0.66, Di = 2 * rc * diF, d = Do / Di, K1 = (1 / Math.PI) * ((d - 1) / d) ** 2 / ((d + 1) / (d - 1) - 2 / Math.log(d)),
    K2 = (6 / Math.PI) * ((d - 1) / Math.log(d) - 1) / Math.log(d), K3 = (3 / Math.PI) * (d - 1) / Math.log(d), C = 4 * E / (1 - nu * nu) * t / (K1 * Do * Do);
  const F = (x) => C * x * ((h0 - x) * (h0 - x / 2) + t * t), S = (x) => 4 * E / (1 - nu * nu) / (K1 * Do * Do) * x * (K2 * (h0 - x / 2) + K3 * t);
  let Fmax = -Infinity, Fmin = Infinity, xs = [];
  for (let i = 0; i <= 400; i++) { const x = 2 * h0 * i / 400, f = F(x); xs.push([x, f]); if (x <= h0 && f > Fmax) Fmax = f; if (x >= h0 && f < Fmin) Fmin = f; }
  const x2 = xs.find(([x, f]) => x > h0 && f >= 0 && xs.some(([x1, f1]) => x1 < x && f1 < 0))?.[0] ?? 2 * h0, stroke = x2, bistable = h0 / t > 2 * Math.SQRT2 && Fmin < 0;
  const sig = Math.max(...xs.filter(([x]) => x <= x2).map(([x]) => Math.abs(S(x)))), SFfat = 0.6 * su / sig /* < 10⁵ cycles: 0.6 σu */;
  const mRing = Math.PI / 4 * (Do * Do - Di * Di) * t * 7800 + 0.012 /* inner lifter lip */, Fpads = nPad / 2 * padArmK * lift /* pads bear on the lip through their flexure arms (half the ring's pads load it at once) */;
  /* 6th audit: the pads are held on the cam by the CARTRIDGE preload, not only by their flexure arms; holding every pad clear of the whole cam excursion means carrying k_f·(δ₀ + 2a + c) per cartridge */ const aCam = camAmp * kLev, kPad = kf * 1000 / (kLev * kLev), Fcart = nCart * (Fpeak / kLev + kPad * (2 * aCam + 0.0013)) /* per pad: peak lamella force through the ×k lever + the pad-level flexure stiffness over the whole cam excursion at the cam (2a·k) + clearance */, Fhold = -Fmin, Fdist = Fpads + Fcart + mRing * aVib, SFhold = Fhold / Math.max(1e-9, Fdist);
  const Fsnap = Fmax + Fpads /* snap to sail against the pad arms */, FhandSail = 2 * Fsnap * branchCap / branchRatio / etaLever, clearance = stroke - lift;
  return { Fcart, aCam, kPad, location: 'undefined: an axle-side ring would rub the rotating pads, a wheel-side ring needs a rotating cable interface', Do, Di, t, h0, ratio: h0 / t, bistable, stroke, Fmax, Fmin, Fsnap, FhandSail, sigMax: sig, SFfatigue: SFfat, cycles: nCycles, mass: mRing, Fhold, Fdist, SFhold, lift, clearance,
    ok: false /* no genuine location (see location) and the cartridge preload far exceeds the hold */ && bistable && FhandSail <= 60 && SFfat >= 2 && SFhold >= 2 && clearance >= 0.0005 };
}
