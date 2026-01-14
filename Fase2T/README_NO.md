# LineFollower - ESP32 Linjefølger

## Rask start

1. **Koble ESP32 via USB** og kjør:
```bash
pio run -e esp32dev -t upload
pio device monitor -b 115200
```

2. **Legg roboten på banen**. Den kalibrerer automatisk ved å snu seg vekselvis over linjen (~10 sekunder).

3. **Roboten kjører** med standard PID-verdier: Kp=1.0, Ki=0.04, Kd=7.0.

## Live PID-tuning via seriell

Du kan endre PID-verdier **underveis** uten å re-kompilere:

- **Endre verdier:**
  ```
  SETPID 0.9 0.03 6.5
  ```
  (format: `SETPID <Kp> <Ki> <Kd>`)

- **Se alle kommandoer:**
  ```
  HELP
  ```

## Tips for tuning

Hvis roboten **svinger for mye på rette linjer**:
- Senk **Kd** (derivativ) – prøv f.eks. `SETPID 1.0 0.04 5.5`
- Senk **Kp** (proporsjonell) – prøv f.eks. `SETPID 0.8 0.04 7.0`

Hvis roboten **er treg til å korrigere ved kurver**:
- Øk **Kp** – prøv f.eks. `SETPID 1.2 0.04 7.0`

Hvis roboten **drifter av linjen over tid**:
- Øk **Ki** (integral) forsiktig – prøv f.eks. `SETPID 1.0 0.06 7.0`
- OBS: Ki er begrenset til [-5000, 5000] for stabilititet.

## Hva kalibrer automatisk

Under oppstart:
- Motorene snur seg vekselvis venstre/høyre (turnSpeed=120, ~10 ms per steg)
- QTR-sensorene registrerer min/max verdier over linjen
- Etter 400 iterasjoner stopper motorene og viser kalibreringsstatus

Gi roboten litt plass rundt linjen slik at flere sensorer kan passere over den under kalibrering.

## Sensoroppsett

- **11 QTR-sensorer** (RC-modus)
- **GPIO-pinner:** 33, 25, 26, 27, 14, 12, 13, 23, 22, 21, 19
- **Emitter pin:** 255

## Motor & PWM

- **Motor A (høyre):** AIN1=GPIO16, AIN2=GPIO17, PWMA=GPIO4
- **Motor B (venstre):** BIN1=GPIO5, BIN2=GPIO15, PWMB=GPIO32
- **PWM:** 5 kHz, 8-bit (0–255)
- **Base-fart:** 200, **Max-fart:** 250

## Klasseoppsett

Koden bruker **klasser** for ryddig struktur:
- `LineFollower` – hovedkontroller
- `Motor` – motorstyrring
- `LineSensor` – QTR-sensor håndtering
- `PIDController` – PID-regulering

Alle kilder ligger i `src/`, headere i `include/`.

