# Fed-DCSA Parameter Tuning Guide

Reference for selecting and tuning parameters for the Fed-DCSA algorithm
(Algorithm 1 from the paper). All formulas reference Corollary 2 and Condition (20).

---

## 1. Parameter Definitions

### Physical Parameters

| Symbol | Name | Unit | Meaning |
|--------|------|------|---------|
| `E_max` | Full battery capacity | Wh | Energy in a freshly swapped drone |
| `E_thr` | Safety reserve threshold | Wh | Algorithmic safety buffer; constraint activates when battery approaches this |
| `e_i` | Energy cost per round | Wh | Energy station _i_ consumes per algorithm round at full activity (x=1) |
| `q_i` | Surveillance weight | — | Quality priority for station _i_; higher = more important to keep active |

### Algorithm Parameters

| Symbol | Name | Meaning |
|--------|------|---------|
| `K` | Outer rounds | Total communication rounds between server and stations |
| `T` | Inner steps | Local projected gradient steps per round |
| `N = K*T` | Total iterations | Total gradient steps across all rounds |
| `tau` | Rounding threshold | Binary decision: x_i >= tau => stay active, x_i < tau => swap |
| `delta` | Probability parameter | Controls tolerance (higher delta = tighter tolerance) |

### Derived Constants (Corollary 2)

| Symbol | Formula | Meaning |
|--------|---------|---------|
| `D_x` | 1.0 | Diameter of decision set [0, 1] |
| `mu` | 1.0 | Strong convexity of Euclidean mirror map |
| `M_bar` | sqrt(sum_i max(q_i, e_i)^2) | Worst-case subgradient norm bound |
| `L_G` | sqrt(sum_i e_i^2) | Lipschitz constant of aggregate constraint G |
| `gamma` | D_x / (M_bar * sqrt(N)) | Step size |
| `eta` | 4 * M_bar * D_x / (delta * sqrt(N)) | Tolerance for gate decision |
| `r_drift` | (2 * L_G * M_bar / mu) * T * gamma | Drift budget (accounts for T inner steps) |

The gate fires feasible when: `G_tilde <= eta - r_drift`

---

## 2. Dimensionless Ratios

The algorithm behavior depends **entirely** on dimensionless ratios, not absolute values.

### The Three Key Ratios

| Ratio | Formula | Controls |
|-------|---------|----------|
| **Drain fraction** | rho_i = e_i / E_max | Swap frequency: rounds per cycle = (1 - alpha) / rho_i |
| **Reserve fraction** | alpha = E_thr / E_max | Safety margin between constraint activation and actual swap |
| **Priority ratio** | sigma_i = q_i / e_i | Condition (20) flexibility and M_bar/L_G ratio |

### Scaling Law

If you multiply **all** energy quantities (E_max, E_thr, e_i, q_i) by the same factor `c`:
- M_bar scales by c, gamma scales by 1/c, overshoot scales by c, E_thr scales by c
- **All ratios are preserved** => identical round-by-round behavior
- Only the battery axis labels change

This means E_max = 1 Wh or E_max = 10000 Wh can produce the same algorithm dynamics
as long as e_i, q_i, E_thr are scaled proportionally.

### What Each Ratio Does

**Drain fraction (rho_i = e_i / E_max):**
- Determines how many rounds a drone operates before needing a swap
- Rounds per battery cycle ~ (1 - alpha) / rho_i
- Higher rho_i = faster drain = more frequent swaps = steeper battery sawtooth
- Also sets L_G through e_i values; higher drain rates make the gate more conservative

**Reserve fraction (alpha = E_thr / E_max):**
- Sets where the constraint g_i first activates (at battery = E_thr)
- Higher alpha = earlier constraint activation = more safety margin
- But also means shorter active cycles per drone = more swaps over K rounds
- Does NOT affect gamma, eta, M_bar, or any convergence parameters

**Priority ratio (sigma_i = q_i / e_i):**
- Must be > 1 for Condition (20) to allow T > 1/(4*delta)
- Higher sigma increases M_bar relative to L_G, which relaxes Condition (20)
- BUT also increases M_bar, which makes gamma smaller and increases overshoot
- Represents physical reality: surveillance value should exceed energy cost

---

## 3. Theoretical Conditions

### Condition (20): Inner Step Limit

```
T < mu * M_bar / (4 * delta * L_G)
```

With mu = 1:

```
T < M_bar / (4 * delta * L_G)
```

**Special cases:**
- When q_i < e_i for all stations: M_bar = L_G, so T < 1/(4*delta).
  With delta=0.1, this gives T < 2.5. Very restrictive.
- When q_i > e_i for all stations: M_bar > L_G, bound relaxes.
  The ratio M_bar/L_G > 1 provides extra room.

**What happens if violated:** Theorem 1 guarantees no longer formally apply.
The algorithm still runs but convergence rates and constraint satisfaction
guarantees from the paper don't hold.

### Safety Condition: Battery Stays Positive

```
alpha > (1 - tau) * M_tilde * sqrt(K) / sqrt(T)
```

where `M_tilde = M_bar / E_max` (normalized subgradient bound).

Equivalently, in absolute terms:

```
E_thr > (1 - tau) * M_bar * sqrt(K) / sqrt(T)
```

This says E_thr must be large enough to absorb the "overshoot" — the battery
drain that occurs between when the constraint activates and when the swap
actually happens (see Section 5).

**What happens if violated:** Battery goes negative before the algorithm can
push x below tau. The drone runs out of energy before swapping.

### Gate Headroom: Gate Can Actually Fire

```
eta - r_drift > 0
```

Substituting:

```
4 * M_bar * D_x / (delta * sqrt(N)) > (2 * L_G * M_bar / mu) * T * D_x / (M_bar * sqrt(N))
```

Simplifies to:

```
4 / delta > 2 * L_G * T / mu
```

Or: `T < 2 * mu / (delta * L_G)`. This is automatically satisfied when
Condition (20) holds (it's a weaker requirement).

**What happens if violated:** The gate threshold eta - r_drift <= 0, meaning
even G_tilde = 0 can't pass the gate. The algorithm is stuck in perpetual
infeasible mode.

---

## 4. Key Tradeoffs

| Tradeoff | Tension |
|----------|---------|
| **N (= K*T) size** | Larger N => smaller eta (gate can fire) and smaller gamma (slower response). Need N large enough for gate to work, but not so large that gamma can't keep up with battery drain. |
| **T (inner steps)** | More T => fewer communication rounds for same N, BUT more drift per round (r_drift grows with T) and Condition (20) limits T. |
| **q_i/e_i ratio** | Higher ratio => relaxes Condition (20) (can use larger T), BUT increases M_bar => smaller gamma => larger overshoot. |
| **alpha (E_thr/E_max)** | Higher alpha => more safety margin at swap time, BUT shorter active cycles => more total swaps. |
| **tau (threshold)** | Higher tau => swap triggers sooner (safer, less overshoot) BUT more frequent swaps. Lower tau => fewer swaps but riskier. |
| **delta** | Higher delta => tighter tolerance eta (gate fires more easily), BUT Condition (20) bound shrinks (smaller T allowed). |

### The Fundamental Tension

The step size gamma = 1/(M_bar * sqrt(N)) must be large enough for the
algorithm to push x from 1.0 to below tau before the battery runs out.
But gamma shrinks as N grows, while battery drains at a fixed physical rate.

The resolution: scale e_i (and q_i) down relative to E_max so that the
physical drain rate is slow enough for the algorithm's step size to respond.

---

## 5. The Overshoot Formula

When battery hits E_thr, the constraint g_i activates and the gate starts
firing infeasible. But x doesn't instantly drop to 0 — it takes multiple
rounds of infeasible gradient steps to push x below tau.

### Rounds to reach swap threshold

During infeasible rounds, x decreases by gamma * e_i per inner step.
Per round (T inner steps): delta_x = T * gamma * e_i

Rounds needed to push x from 1.0 to tau:

```
rounds_to_swap = (1 - tau) / (T * gamma * e_i)
```

### Battery drained during those rounds

```
overshoot = rounds_to_swap * e_i
         = (1 - tau) / (T * gamma * e_i) * e_i
         = (1 - tau) / (T * gamma)
```

**The e_i cancels out!** All stations experience the same absolute overshoot
regardless of their individual drain rates.

### In terms of input parameters

```
overshoot = (1 - tau) / (T * gamma)
          = (1 - tau) * M_bar * sqrt(N) / T
          = (1 - tau) * M_bar * sqrt(K * T) / T
          = (1 - tau) * M_bar * sqrt(K) / sqrt(T)
```

### Effective battery at swap

```
battery_at_swap = E_thr - overshoot
```

This must be positive for the algorithm to be safe.

### Example (realistic parameter set)

- tau = 0.7, M_bar = 0.5612, K = 20000, T = 5
- overshoot = 0.3 * 0.5612 * sqrt(20000) / sqrt(5) = 0.3 * 0.5612 * 63.25 = 10.65 Wh
- battery_at_swap = 20 - 10.65 = 9.35 Wh (observed: ~9.5 Wh)

---

## 6. Quick Tuning Checklist

### Step 1: Fix physical parameters

Choose E_max and E_thr based on your actual drone hardware.

- alpha = E_thr / E_max should be 15-25% (typical safety reserve)

### Step 2: Choose drain and priority rates

Set e_i and q_i such that:
- **q_i > e_i** for all stations (needed for Condition 20 flexibility)
- **q_i/e_i ~ 2-3** is a good starting point
- **e_i/E_max ~ 0.001-0.002** gives ~500-800 rounds per battery cycle

### Step 3: Compute M_bar and L_G

```
M_bar = sqrt(sum(max(q_i, e_i)^2))    # = sqrt(sum(q_i^2)) when q_i > e_i
L_G   = sqrt(sum(e_i^2))
```

### Step 4: Choose delta and compute T bound

- delta = 0.1 is a good default
- T_max = M_bar / (4 * delta * L_G)
- Choose T < T_max (integer). T = 3-5 is typical.

### Step 5: Choose K

- N = K * T should be large enough for meaningful convergence
- Larger K = more rounds = more swap events
- Check safety: overshoot = (1 - tau) * M_bar * sqrt(K) / sqrt(T) < E_thr

### Step 6: Verify all conditions

1. **Condition (20):** T < M_bar / (4 * delta * L_G) ?
2. **Safety:** E_thr > (1 - tau) * M_bar * sqrt(K) / sqrt(T) ?
3. **Gate headroom:** eta - r_drift > 0 ?
4. **Positive battery:** battery_at_swap = E_thr - overshoot > 0 ?

### Step 7: Estimate swap frequency

Per station: swaps ~ K * rho_i / (1 - alpha)
Total swaps ~ K * sum(rho_i) / (1 - alpha)

---

## 7. Working Example

### Parameters (validated against experiment)

```
Physical:  E_max = 100 Wh, E_thr = 20 Wh (alpha = 0.20)
Drain:     e_i = [0.10, 0.12, 0.08, 0.15] Wh/round
Priority:  q_i = [0.25, 0.30, 0.20, 0.35]
Algorithm: K = 20000, T = 5, tau = 0.7, delta = 0.1
```

### Derived constants

```
M_bar   = sqrt(0.25^2 + 0.30^2 + 0.20^2 + 0.35^2) = 0.5612
L_G     = sqrt(0.10^2 + 0.12^2 + 0.08^2 + 0.15^2) = 0.2309
N       = 100,000
gamma   = 1 / (0.5612 * 316.23)         = 0.005634
eta     = 4 * 0.5612 / (0.1 * 316.23)   = 0.0710
r_drift = 2 * 0.2309 * 0.5612 * 5 * 0.005634 = 0.0073
```

### Condition checks

```
Condition (20):  T < 0.5612 / (4 * 0.1 * 0.2309) = 6.08
                 T = 5 < 6.08  =>  SATISFIED

Safety:          overshoot = 0.3 * 0.5612 * sqrt(20000) / sqrt(5) = 10.65
                 E_thr = 20 > 10.65  =>  SATISFIED
                 battery_at_swap = 20 - 10.65 = 9.35 Wh > 0

Gate headroom:   eta - r_drift = 0.0710 - 0.0073 = 0.0637 > 0  =>  SATISFIED
```

### Expected vs actual results

| Metric | Expected | Actual |
|--------|----------|--------|
| battery_at_swap | ~9.35 Wh | 9.36-9.55 Wh |
| Station 3 cycle length | ~533 rounds | ~604 rounds |
| Station 2 cycle length | ~1000 rounds | ~1134 rounds |
| Total swaps | ~99 | 98 |
| Feasible rounds | ~65% | 65.2% |

The cycle lengths are longer than the simple (1-alpha)/rho_i estimate because
they include the ~70 infeasible response rounds per swap event.

### Dimensionless ratios

```
rho_i  = [0.0010, 0.0012, 0.0008, 0.0015]
alpha  = 0.20
sigma_i = [2.5, 2.5, 2.5, 2.33]
```
