import tkinter as tk
import tkinter.tix as tix
import serial

root = tix.Tk()
root.geometry("400x400")

ser = serial.Serial(
    port="COM7",
    baudrate=19200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=None)

data = []

def write_values_to_list():
    data.append(gainSel.get())
    data.append(resolutionX.get())
    data.append(resolutionY.get())
    data.append(resolutionZ.get())
    data.append(digitalFilter.get())
    data.append(osr.get())
    # data.append(givenFieldEntry.get())

def send_read_VM_request():
    """Send data over UART with read volatile memory (current calibration) request (MODE: 0)"""
    write_values_to_list()
    data.append(0)
    for i in data:
        ser.write(bytes(data))
    data.clear()
    received_data = ''
    flag = True
    while 1:
        if flag:
            received_data = ser.read(20)
            if received_data != '':
                print(received_data.decode())
                flag = False
                received_data = ''
        else:
            received_data = ser.read(122)
            if received_data != '':
                print(received_data.decode())
                break

def send_write_NVM_request():
    """Send data over UART with write non-volatile memory request (MODE: 1)"""
    write_values_to_list()
    data.append(1)
    for i in data:
        ser.write(bytes(data))
    data.clear()
    received_data = ''
    flag = True
    while 1:
        if flag:
            received_data = ser.read(49)
            if received_data != '':
                print(received_data.decode())
                flag = False
                received_data = ''
        else:
            received_data = ser.read(43)
            if received_data != '':
                print(received_data.decode())
                break


def send_calibration_request():
    """Send data over UART with calibration request (MODE: 2)"""
    write_values_to_list()
    data.append(2)
    for i in data:
        ser.write(bytes(data))
    data.clear()
    received_data = ''
    flag = True
    while 1:
        if flag:
            received_data = ser.read(26)
            if received_data != '':
                print(received_data.decode())
                flag = False
                received_data = ''
        else:
            received_data = ser.read(35)
            if received_data != '':
                print(received_data.decode())
                break


# Configure grid

for i in range(9):
    tk.Grid.rowconfigure(root, i, weight=1)

for i in range(3):
    tk.Grid.columnconfigure(root, i, weight=1)

# List of options
bits0_1 = [0, 1, 2, 3]
bits0_2 = [0, 1, 2, 3, 4, 5, 6, 7]
bits0_3 = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

# Labels
myLabel = tk.Label(root, text="MLX90393 Configurator", font="Arial 12 bold")
gainSelLabel = tk.Label(root, text="GAIN_SEL")
resolutionXLabel = tk.Label(root, text="RES_X")
resolutionYLabel = tk.Label(root, text="RES_Y")
resolutionZLabel = tk.Label(root, text="RES_Z")
digFilterLabel = tk.Label(root, text="DIG_FILTER")
osrLabel = tk.Label(root, text="OSR")
givenFieldLabel = tk.Label(root, text="Field intensity [Oe]:")

# Entries
givenFieldEntry = tk.Entry(root, width=5)

# Dropdowns
gainSel = tk.IntVar()
gainSel.set(bits0_2[0])
gainSelDropDown = tk.OptionMenu(root, gainSel, *bits0_2)

resolutionX = tk.IntVar()
resolutionX.set(bits0_1[0])
resolutionXDropDown = tk.OptionMenu(root, resolutionX, *bits0_1)

resolutionY = tk.IntVar()
resolutionY.set(bits0_1[0])
resolutionYDropDown = tk.OptionMenu(root, resolutionY, *bits0_1)

resolutionZ = tk.IntVar()
resolutionZ.set(bits0_1[0])
resolutionZDropDown = tk.OptionMenu(root, resolutionZ, *bits0_1)

digitalFilter = tk.IntVar()
digitalFilter.set(bits0_2[0])
digitalFilterDropDown = tk.OptionMenu(root, digitalFilter, *bits0_2)

osr = tk.IntVar()
osr.set(bits0_1[0])
osrDropDown = tk.OptionMenu(root, osr, *bits0_1)

# # Checkboxes
# digFilter = tk.BooleanVar()
# digFilterCheckbox = tk.Checkbutton(root, variable=digFilter)

# Buttons
startButton = tk.Button(root, text="Start Configuration", bd=10, width=10, bg="Green", font="Helvetica 10 bold",
                        relief="flat", wraplength=90, cursor="cross", command=send_calibration_request)
read_currentButton = tk.Button(root, text="Read current configuration from VM", bd=10, width=10, bg="LightBlue3",
                               relief="flat", wraplength=90, cursor="cross", command=send_read_VM_request)
save_configButton = tk.Button(root, text="Save current configuration in NVM", bd=10, width=10, bg="red",
                              relief="flat", wraplength=90, cursor="cross", command=send_write_NVM_request)

# Tooltips and bindings
tooltip = tix.Balloon(root)
tooltip.bind_widget(gainSelLabel, balloonmsg="Analog chain gain setting, factor 5 between min and max code")
# resolutionXToolTip = tix.Balloon(root)
tooltip.bind_widget(resolutionXLabel, balloonmsg="Selects the desired 16-bit output value from the 19-bit ADC")
tooltip.bind_widget(resolutionYLabel, balloonmsg="Selects the desired 16-bit output value from the 19-bit ADC")
tooltip.bind_widget(resolutionZLabel, balloonmsg="Selects the desired 16-bit output value from the 19-bit ADC")
tooltip.bind_widget(digFilterLabel, balloonmsg="Digital filter applicable to ADC")
tooltip.bind_widget(osrLabel, balloonmsg="Magnetic sensor ADC oversampling ratio")
tooltip.bind_widget(givenFieldLabel, balloonmsg="Magnetic field intensity in which sensor is placed")
tooltip.bind_widget(startButton, balloonmsg="Start sensor calibration process according to your settings")
tooltip.bind_widget(read_currentButton, balloonmsg="Read current sensor configuration from volatile memory")
tooltip.bind_widget(save_configButton, balloonmsg="Save current sensor configuration in non-volatile memory")

# Grids
myLabel.grid(row=1, column=2)
gainSelLabel.grid(row=2, column=0)
resolutionXLabel.grid(row=3, column=0)
resolutionYLabel.grid(row=4, column=0)
resolutionZLabel.grid(row=5, column=0)
digFilterLabel.grid(row=6, column=0)
osrLabel.grid(row=7, column=0)
# givenFieldLabel.grid(row=3, column=2)

# givenFieldEntry.grid(row=4, column=2)
gainSelDropDown.grid(row=2, column=1)

resolutionXDropDown.grid(row=3, column=1)
resolutionYDropDown.grid(row=4, column=1)
resolutionZDropDown.grid(row=5, column=1)
digitalFilterDropDown.grid(row=6, column=1)
osrDropDown.grid(row=7, column=1)

startButton.grid(row=5, column=2)
read_currentButton.grid(row=6, column=2)
save_configButton.grid(row=7, column=2)

root.mainloop()
