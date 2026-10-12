import re
import csv
import time
import threading

import serial
import matplotlib.pyplot as plt

# CONFIGURACION

PUERTO = "COM5"
BAUDIOS = 9600
ARCHIVO_CSV = "registro.csv"

# EXPRESIONES REGULARES
RE_TEMP = re.compile(r"Temp=(\d+)\s*C\s*\|\s*Accion=(\w+\s?\w*)")
RE_PM = re.compile(r"PM=(\d+)\s*C")
RE_ERR = re.compile(r"MSG,ERROR DHT11 codigo=(\d+)")

ACCIONES = ["CALEF ON", "REPOSO", "FAN BAJA", "FAN MEDIA", "FAN ALTA"]

# DATOS
t_temp, v_temp = [], []
t_acc, v_acc = [], []
t_pm, v_pm = [], []

pm_actual = 20
t0 = None
datos_nuevos = threading.Event()

def ahora():
    return time.time() - t0

# CONSOLA

def consola(ser, detener):
    esperando_pm = False

    while not detener.is_set():
        try:
            linea = input()
        except (EOFError, OSError):
            break

        comando = linea.strip()

        # q es local y nunca se envia al microcontrolador.
        if comando.lower() == "q":
            detener.set()
            break

        if not comando:
            continue

        if not esperando_pm:
            # Validacion ESTRICTA: (solamente "P" o "p")       
            if comando == "P" or comando == "p":
                ser.write(b"P")
                ser.flush()
                esperando_pm = True
                print("[TX] P")
                print("[LOCAL] Sensado pausado por el firmware.")
                print("[LOCAL] Ingrese un PM de dos cifras entre 10 y 25 C:")
            else:
                print("[LOCAL] Entrada invalida.")
                print("[LOCAL] Para modificar el PM debe ingresar UNICAMENTE P o p.")
        else:
            # Luego de P, solo se acepta un PM valido de dos cifras.
            if comando.isdigit() and len(comando) == 2:
                valor = int(comando)

                if 10 <= valor <= 25:
                    ser.write(comando.encode("ascii"))
                    ser.flush()
                    print(f"[TX] {comando}")
                    print("[LOCAL] PM enviado. El firmware reanudara el sensado.")
                    esperando_pm = False
                else:
                    print("[LOCAL] PM invalido. Debe estar entre 10 y 25 C.")
                    print("[LOCAL] Ingrese nuevamente el PM:")
            else:
                print("[LOCAL] Debe ingresar exactamente dos cifras (10 a 25).")

# RECEPCION Y PARSEO
def procesar_linea(linea):
    global pm_actual

    m = RE_TEMP.search(linea)
    if m:
        temp = int(m.group(1))
        accion = m.group(2).strip()

        if accion not in ACCIONES:
            return

        t = ahora()
        t_temp.append(t)
        v_temp.append(temp)
        t_acc.append(t)
        v_acc.append(ACCIONES.index(accion))

        datos_nuevos.set()
        return

    m = RE_PM.search(linea)
    if m:
        pm_actual = int(m.group(1))
        t_pm.append(ahora())
        v_pm.append(pm_actual)
        datos_nuevos.set()
        return

    # Los errores y demas mensajes ya se muestran de forma transparente
    # en la terminal. No hace falta volver a imprimirlos aqui.
    m = RE_ERR.search(linea)
    if m:
        return

# CSV
def guardar_csv():
    with open(ARCHIVO_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tiempo_s", "temperatura_C", "PM_C", "accion"])

        for t, temp, nivel in zip(t_temp, v_temp, v_acc):
            pm = v_pm[0] if v_pm else pm_actual
            for tp, vp in zip(t_pm, v_pm):
                if tp <= t:
                    pm = vp
                else:
                    break

            w.writerow([
                f"{t:.1f}",
                temp,
                pm,
                ACCIONES[nivel]
            ])

    print(f"Registro guardado en {ARCHIVO_CSV}")

# GRAFICO EN TIEMPO REAL
def crear_grafico():
    plt.ion()

    fig, (ax_temp, ax_acc) = plt.subplots(
        2, 1, figsize=(11, 8), sharex=True,
        gridspec_kw={"height_ratios": [2, 1]}
    )

    # Grafico de temperatura
    linea_temp, = ax_temp.plot(
        [], [], "o-", label="Temperatura", markersize=4
    )
    linea_pm, = ax_temp.step(
        [], [], where="post", linestyle="--",
        linewidth=2, label="Punto medio (PM)"
    )
    linea_inf, = ax_temp.step(
        [], [], where="post", linestyle=":",
        linewidth=1.5, label="Limite inferior"
    )
    linea_sup, = ax_temp.step(
        [], [], where="post", linestyle=":",
        linewidth=1.5, label="Limite superior"
    )

    ax_temp.set_ylabel("Temperatura (C)")
    ax_temp.set_title("Control de temperatura - Tiempo real")
    ax_temp.grid(True, alpha=0.3)
    ax_temp.legend(loc="upper left")

    # Grafico de acciones
    linea_acc, = ax_acc.step(
        [], [], where="post", linewidth=2
    )

    ax_acc.set_yticks(range(len(ACCIONES)))
    ax_acc.set_yticklabels(ACCIONES)
    ax_acc.set_ylim(-0.5, len(ACCIONES) - 0.5)
    ax_acc.set_xlabel("Tiempo (s)")
    ax_acc.set_ylabel("Accion")
    ax_acc.grid(True, axis="x", alpha=0.3)

    fig.tight_layout()
    fig.show()

    return fig, ax_temp, ax_acc, linea_temp, linea_pm, linea_inf, linea_sup, linea_acc


def actualizar_grafico(fig, ax_temp, ax_acc,
                       linea_temp, linea_pm, linea_inf, linea_sup, linea_acc):

    if not t_temp:
        return

    # Temperatura
    linea_temp.set_data(t_temp, v_temp)

    # Para que las lineas escalonadas de PM lleguen hasta el instante actual.
    tiempo_final = max(t_temp[-1], ahora())

    pm_t = list(t_pm)
    pm_v = list(v_pm)

    if pm_t:
        if pm_t[-1] < tiempo_final:
            pm_t.append(tiempo_final)
            pm_v.append(pm_v[-1])

        limite_inf = [pm - 5 for pm in pm_v]
        limite_sup = [pm + 5 for pm in pm_v]

        linea_pm.set_data(pm_t, pm_v)
        linea_inf.set_data(pm_t, limite_inf)
        linea_sup.set_data(pm_t, limite_sup)

    # Accion
    linea_acc.set_data(t_acc, v_acc)

    # Ajuste automatico de los ejes
    xmax = max(30.0, tiempo_final + 5.0)
    ax_temp.set_xlim(0, xmax)

    valores_y = list(v_temp)
    if pm_v:
        valores_y.extend([x - 5 for x in pm_v])
        valores_y.extend([x + 5 for x in pm_v])

    ymin = min(valores_y) - 3
    ymax = max(valores_y) + 3

    # Evita un eje demasiado pequeno cuando recien comienza.
    ymin = min(ymin, 10)
    ymax = max(ymax, 30)

    ax_temp.set_ylim(ymin, ymax)

    fig.canvas.draw_idle()
    fig.canvas.flush_events()
    plt.pause(0.001)

# GUARDAR IMAGENES FINALES
def guardar_graficos(fig):
    # Imagen conjunta en tiempo real
    fig.savefig("grafico_tiempo_real.png", dpi=150)

    # Ademas conserva los nombres usados anteriormente.
    fig_temp, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t_temp, v_temp, "o-", label="Temperatura", markersize=4)

    if t_pm:
        pm_t = list(t_pm)
        pm_v = list(v_pm)

        if t_temp and pm_t[-1] < t_temp[-1]:
            pm_t.append(t_temp[-1])
            pm_v.append(pm_v[-1])

        limite_inf = [pm - 5 for pm in pm_v]
        limite_sup = [pm + 5 for pm in pm_v]

        ax.step(pm_t, pm_v, where="post", linestyle="--",
                linewidth=2, label="Punto medio (PM)")
        ax.step(pm_t, limite_inf, where="post", linestyle=":",
                linewidth=1.5, label="Limite inferior")
        ax.step(pm_t, limite_sup, where="post", linestyle=":",
                linewidth=1.5, label="Limite superior")
        ax.fill_between(
            pm_t, limite_inf, limite_sup,
            step="post", alpha=0.12, label="Rango ideal"
        )

    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Temperatura (C)")
    ax.set_title("Temperatura medida y punto medio de confort")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig_temp.tight_layout()
    fig_temp.savefig("grafico_temperatura.png", dpi=150)
    plt.close(fig_temp)

    fig_acc, ax = plt.subplots(figsize=(10, 4))
    ax.step(t_acc, v_acc, where="post", linewidth=2)
    ax.fill_between(t_acc, v_acc, step="post", alpha=0.2)

    ax.set_yticks(range(len(ACCIONES)))
    ax.set_yticklabels(ACCIONES)
    ax.set_ylim(-0.5, len(ACCIONES) - 0.5)
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Accion")
    ax.set_title("Accion del sistema en el tiempo")
    ax.grid(True, axis="x", alpha=0.3)
    fig_acc.tight_layout()
    fig_acc.savefig("grafico_acciones.png", dpi=150)
    plt.close(fig_acc)

    print("Graficos guardados:")
    print("  grafico_tiempo_real.png")
    print("  grafico_temperatura.png")
    print("  grafico_acciones.png")

# PRINCIPAL

def main():
    global t0

    try:
        ser = serial.Serial(PUERTO, BAUDIOS, timeout=0.1)
    except serial.SerialException as e:
        print(f"No se pudo abrir {PUERTO}: {e}")
        print("Verifica que el Monitor Serie este cerrado.")
        return

    time.sleep(2)

    detener = threading.Event()
    hilo = threading.Thread(
        target=consola,
        args=(ser, detener),
        daemon=True
    )
    hilo.start()

    t0 = time.time()

    # PM inicial del firmware.
    t_pm.append(0.0)
    v_pm.append(pm_actual)

    (fig, ax_temp, ax_acc,
     linea_temp, linea_pm,
     linea_inf, linea_sup,
     linea_acc) = crear_grafico()

    buffer = ""

    print("=" * 60)
    print(" GRAFICOS EN TIEMPO REAL")
    print("=" * 60)
    print(" Menu:")
    print("   'P' o 'p' + Enter -> detener sensado y solicitar cambio de PM")
    print("   10 a 25     -> nuevo PM, (solo despues de P/p)")
    print("   q + Enter   -> cerrar el programa")
    print("=" * 60)
    print()

    try:
        while not detener.is_set():
            # Si el usuario cierra la ventana del grafico, terminamos.
            if not plt.fignum_exists(fig.number):
                detener.set()
                break

            byte = ser.read(1)

            if byte:
                c = byte.decode("ascii", errors="ignore")

                if c == "\n":
                    linea = buffer.strip()
                    buffer = ""
                    if linea:
                        # Terminal serial transparente: mostramos TODO lo
                        # recibido por UART, no solo las lineas de temperatura.
                        print(linea, flush=True)
                        procesar_linea(linea)
                elif c != "\r":
                    buffer += c

            # Solo redibuja cuando llegaron datos nuevos.
            if datos_nuevos.is_set():
                actualizar_grafico(
                    fig, ax_temp, ax_acc,
                    linea_temp, linea_pm,
                    linea_inf, linea_sup,
                    linea_acc
                )
                datos_nuevos.clear()
            else:
                # Mantiene responsiva la ventana de Matplotlib.
                plt.pause(0.01)

    except KeyboardInterrupt:
        detener.set()

    finally:
        if ser.is_open:
            ser.close()

        print("\nCerrando...")
        guardar_csv()

        if plt.fignum_exists(fig.number):
            actualizar_grafico(
                fig, ax_temp, ax_acc,
                linea_temp, linea_pm,
                linea_inf, linea_sup,
                linea_acc
            )
            guardar_graficos(fig)

        plt.ioff()
        plt.close("all")


if __name__ == "__main__":
    main()
