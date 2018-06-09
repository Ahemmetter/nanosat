import numpy as np
from matplotlib import pyplot as plt

# --- Plot colors ---

flat_black = "#262626"
flat_brown = "##523B2E"
maroon = "#682624"
flat_red = "#C43730"
flat_orange = "#D6521E"
flat_yellow = "#FFCB46"
lime = "#8DAF41"
forest_green = "#2B503A"
flat_green = "#00CC7B"
mint = "#00A088"
sky_blue = "#1E80B3"
flat_blue = "#50669D"
flat_purple = "#755FBE"
magenta = "#9046A7"
plum = "#502B4D"

# --- Solar Power ---

# constants
T = 90                                      # min (orbital period)
i0 = 1353/1e6                               # W/mm**2 (solar intensity)
l = 54.5                                    # mm (length of solar cell)
b = 38                                      # mm (width of solar cell)
#eta = 11.7                                 # % (efficiency of solar cell)
#FF = 70                                    # % (fill factor of solar cell)
Pmax = 220                                  # mW (maximum solar cell power)

i15 = 1.1 * i0 * 0.7**(1.5**0.678)          # W/mm**2 (intensity at 1.5 AM)
A = l*b                                     # mm**2 (area of solar cell)
Prad = i0*A/1000.0                          # mW (power incident on solar cell)

# relative area depends on angle during orbit
# angles are in seconds (deg)
step = 1.0/60
angles = np.arange(0, 360, step)

# calculate power coming from solar cell depending on the angle
def Psolar(angle):
    if np.sin(np.deg2rad(angle)) > 0:
        return Pmax * np.sin(np.deg2rad(angle))
    else:
        return 0

Psun = []
for angle in angles:
    Psun.append(Psolar(angle))

# convert angle to time in orbit
def deg2time(angle):
    time = 90.0/360 * angle
    return time

# convert time in orbit to angle
def time2deg(time):
    angle = time/90.0 * 360
    return angle

# --- Power Consumption ---

# things that always need power (idle consumption in mW)
gps_idle = 50                               # GPS
mcu_idle = 10                                # MCU
imu_idle = 6                                # IMU
rxtx_idle = 0.9                             # Transceiver
cam_idle = 0
ac_idle = 0
env_idle = 1
# total idle power consumption is just the sum
Pidle = gps_idle + mcu_idle + imu_idle + rxtx_idle + cam_idle + ac_idle+env_idle

# power consumption that is periodic
# transceiver:
P_rxtx = 200-rxtx_idle                      # mW (maximum power)
T_rxtx = 15                                 # min (length of operation)
start_rxtx = 60                             # at what angle it starts working
end_rxtx = start_rxtx + time2deg(T_rxtx)    # at what angle it stops working
f_rxtx = 360                                # after how many degrees it repeats

# camera:
P_cam = 60-cam_idle                         # mW (maximum power)
T_cam = 0.03                                # min (length of operation)
start_cam = 0                               # at what angle it starts working
end_cam = 180                               # at what angle it stops working
f_cam = 5                                   # after how many degrees it repeats

# attitude correction (AC):
P_ac = 66-ac_idle                           # mW (maximum power)
T_ac = 1                                    # min (length of operation)
start_ac = 0                                # at what angle it starts working
f_ac = 10                                   # after how many degrees it repeats

# switch subsystems on/off
rxtx_active = True
cam_active = True
AC_active = True


# entire power consumption profile over one orbit, no idle
Pactive = np.zeros(len(angles))
for i in range(0, len(angles)):
    # add camera:
    if cam_active:
        if (angles[i] % f_cam >= start_cam) and (angles[i] % f_cam < time2deg(T_cam)) and (angles[i] >= start_cam) and (angles[i] < end_cam):
            Pactive[i] = Pactive[i] + P_cam

    # add Rx/Tx
    if rxtx_active:
        if (angles[i] % f_rxtx >= start_rxtx) and (angles[i] % f_rxtx < end_rxtx) and (angles[i] >= start_rxtx) and (angles[i] < end_rxtx):
            Pactive[i] = Pactive[i] + P_rxtx

    # add attitude control
    if AC_active:
        if (angles[i] % f_ac >= start_ac) and (angles[i] % f_ac < time2deg(T_ac)):
            Pactive[i] = Pactive[i] + P_ac

# total power consumption from instruments
Ptotal = Pidle + Pactive

# --- Battery Charging Cycle ---

Cmax = 200                                  # mAh (capacity of the battery)
Cmaxmin = Cmax*60.0                         # mAmin (capacity in minutes)
charging_speed = 5.0                        # how fast the battery is charged
I_charge = Cmax/charging_speed              # mA (charging current)
U_bat = 3.7                                 # V (battery voltage)
Pcharge = U_bat * I_charge                  # mW (charging power)
Cnew = 0.5*Cmaxmin                          # mAmin (initial battery charge)

C = []                                      # mAmin (current battery charge)
I = []                                      # mA (current flowing current)
level = []

for i in range(0, len(angles)):
    angle = angles[i]
    if Cnew > Cmaxmin:
        I.append(0)
        C.append(Cmaxmin)
    else:
        # if solar delivers more than necessary
        if Psolar(angle) > (Ptotal[i] + Pcharge):
            # then provide power for consumers and charge battery
            Pavail = Psolar(angle) - Ptotal[i]
            if (Pavail/U_bat) >= I_charge:
                # if there is enough to charge the battery at full speed, do that
                Cnew += I_charge * (T/360.0*step)  # charging speed per degree
                I.append(I_charge)
            else:
                # if there is less, charge it at a slower speed
                I_charge_reduced = Pavail/U_bat
                Cnew += I_charge_reduced * (T/360.0*step)   # charging speed per degree
                I.append(I_charge_reduced)
            C.append(Cnew)
            level.append((Cnew/Cmaxmin)*100)

        else:
            # there is not enough from the solar cell to idle
            Pleft = Ptotal[i] - Psolar(angle)
            I_discharge = Pleft/U_bat
            Cnew -= I_discharge * (T/360.0*step)
            I.append(-I_discharge)
            C.append(Cnew)
            level.append((Cnew/Cmaxmin)*100)


fig = plt.figure(figsize = (9, 6))
ax1 = fig.add_subplot(3, 2, 1)
ax1.set_ylabel("power [mW]", color=flat_red)
ax1.plot(angles, Psun, linestyle = "--", color = flat_orange, label="solar output", linewidth=0.5)
ax1.plot(angles, Ptotal, color = flat_red, label="power consumption", linewidth=0.5)

ax2 = fig.add_subplot(3, 2, 3)
ax2.set_ylabel("battery level [%]", color=flat_blue)
ax2.plot(angles, level, color=flat_blue, label="charge level", linewidth=0.5)

ax3 = fig.add_subplot(3, 2, 5)
ax3.set_ylabel("current [mA]", color=flat_yellow)
ax3.plot(angles, I, color=flat_yellow, label="charging speed", linewidth=0.5)
ax3.axhline(y=0, color="k")

ax1.legend(loc="best")
ax2.legend(loc="best")
ax3.legend(loc="best")

ax4 = fig.add_subplot(3, 2, 2, polar=True)
ax4.plot(np.deg2rad(angles), Psun, color = flat_orange, linewidth=0.5)
ax4.plot(np.deg2rad(angles), Ptotal, color = flat_red, linewidth=0.5)

ax5 = fig.add_subplot(3, 2, 4, polar=True)
ax5.plot(np.deg2rad(angles), level, color=flat_blue, linewidth=0.5)

ax6 = fig.add_subplot(3, 2, 6, polar=True)
ax6.plot(np.deg2rad(angles), I, color = flat_yellow, linewidth=0.5)
fig.tight_layout()
plt.show()
