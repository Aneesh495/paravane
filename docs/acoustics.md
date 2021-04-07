# Underwater Acoustics and Sound Velocity Ray Tracing

In seawater, electromagnetic radiation attenuates within tens of meters, leaving acoustic wave propagation as the sole viable physical mechanism for long-range underwater communications, positioning, and seafloor imaging.

## Sound Speed Profiles and Refraction

The speed of sound in seawater $c$ is a thermodynamic function of temperature $T$ (degrees Celsius), salinity $S$ (Practical Salinity Units), and hydrostatic pressure $P$ (or depth $z$). Paravane implements both the UNESCO Chen-Millero (1977) formula and the Mackenzie (1981) 9-term empirical equation:

$$c(T, S, z) = 1448.96 + 4.591 T - 5.304 \times 10^{-2} T^2 + 2.374 \times 10^{-4} T^3 + 1.340 (S - 35) + 1.630 \times 10^{-2} z + 1.675 \times 10^{-7} z^2 - 1.025 \times 10^{-2} T (S - 35) - 7.139 \times 10^{-13} T z^3$$

Because temperature decreases rapidly in the upper thermocline while hydrostatic pressure increases monotonically with depth, sound speed reaches a pronounced minimum at approximately 1000 meters depth. This layer acts as an acoustic waveguide known as the Deep Sound Channel or SOFAR (Sound Fixing and Ranging) channel.

Paravane provides canonical Munk sound velocity profiles:

$$c(z) = c_0 \left[ 1 + \epsilon \left( \tilde{z} - 1 + e^{-\tilde{z}} \right) \right], \quad \tilde{z} = \frac{2(z - z_0)}{B}$$

where $c_0 = 1500 \text{ m/s}$, $\epsilon = 0.00737$, $z_0 = 1000 \text{ m}$, and $B = 1000 \text{ m}$.

## Hamiltonian Acoustic Ray Tracing (Runge-Kutta 4th Order)

According to Snell's law in horizontally stratified media, the ray parameter $\xi$ remains constant along an acoustic wave trajectory:

$$\xi = \frac{\cos \theta(z)}{c(z)} = \text{constant}$$

where $\theta$ is the grazing angle relative to the horizontal plane.

To trace acoustic paths without singular boundary artifacts, Paravane formulates ray propagation as a Hamiltonian system parameterized by arc length $s$:

$$\frac{dx}{ds} = \cos \theta$$

$$\frac{dz}{ds} = \sin \theta$$

$$\frac{d\theta}{ds} = -\frac{1}{c(z)} \frac{dc}{dz} \cos \theta$$

$$\frac{d\tau}{ds} = \frac{1}{c(z)}$$

where $x$ is horizontal range, $z$ is depth, $\theta$ is ray grazing angle, and $\tau$ is acoustic one-way travel time.

These differential equations are integrated numerically using a 4th-order Runge-Kutta (RK4) scheme. Boundary interactions are handled explicitly:
- **Sea Surface ($z \le 0$)**: Specular reflection occurs, inverting vertical velocity ($z \to |z|$) and grazing angle ($\theta \to |\theta|$).
- **Seabed ($z \ge z_{\text{bottom}}$)**: Specular reflection upward ($z \to 2 z_{\text{bottom}} - z$, $\theta \to -|\theta|$).

## Transmission Loss and Sonar Equations

Total acoustic transmission loss $TL$ is modeled by combining spherical/cylindrical geometric spreading with medium absorption:

$$TL(r) = 20 \log_{10}(r) + \alpha(f) \cdot r \times 10^{-3} \quad [\text{dB}]$$

The absorption coefficient $\alpha(f)$ in dB/km is computed via the Francois-Garrison and Thorp empirical equations:

$$\alpha(f) = \frac{0.11 f^2}{1 + f^2} + \frac{44 f^2}{4100 + f^2} + 2.75 \times 10^{-4} f^2 + 0.003 \quad [\text{dB/km}]$$

where $f$ is acoustic frequency in kHz.

## Acoustic Positioning: USBL and LBL

### Ultra-Short Baseline (USBL)
A surface vessel equipped with an acoustic transceiver array measures the phase difference of arrival (PDOA) between hydrophone elements separated by sub-wavelength distances $d < \lambda / 2$. This yields direction cosines $(u, v)$ to the AUV transponder:

$$u = \frac{\Delta \phi_x \cdot c}{2\pi f d}, \quad v = \frac{\Delta \phi_y \cdot c}{2\pi f d}$$

Combining slant range $R = c \cdot \tau / 2$ with surface vessel pitch and roll attitude transformations yields the absolute NED position vector of the subsea vehicle.

### Long Baseline (LBL)
A seafloor transponder array at calibrated geodetic coordinates transmits acoustic interrogation pulses. Paravane solves the nonlinear multilateration system via iterative Gauss-Newton optimization, yielding sub-decimeter positioning fixes and computing Geometric Dilution of Precision (GDOP).
