import tkinter as tk
import serial
from configparser import ConfigParser
from itertools import count
import time
import datetime

class SenderFrame(tk.Frame):
    
    def crc8(data):
        """
        Calculate the CRC8 checksum for the given data.

        Args:
        data (bytes): The data for which the CRC8 is to be calculated.

        Returns:
        int: The CRC8 checksum.
        """
        polynomial = 0x07
        crc = 0x00

        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ polynomial
                else:
                    crc <<= 1
                crc &= 0xFF  # Ensure that CRC remains within 8 bits

        return crc
    
    def __init__(self, master=None, serial_instance=None):
        super().__init__(master)
        self.master = master
        self.serial_instance = serial_instance
        self.pack()

        self.create_widgets()

    def create_widgets(self):
        tk.Button(self, text="Flight", command=lambda: self.send_data("flight")).pack(pady=10)
        tk.Button(self, text="Sleep", command=lambda: self.send_data("sleep")).pack(pady=10)
        tk.Button(self, text="Elace", command=lambda: self.send_data("elace")).pack(pady=10)
        tk.Button(self, text="Reboot", command=lambda: self.send_data("reboot")).pack(pady=10)

        # 新たに追加されるウィジェット
        self.deg_entry = tk.Entry(self)
        self.deg_entry.pack(pady=10)
        tk.Button(self, text="Launcher", command=self.launch_data).pack(pady=10)

    def launch_data(self):
        deg = self.deg_entry.get()
        try:
            deg_value = int(deg)
            if 68 <= deg_value <= 80:
                # 指定された範囲内の値の場合、シリアルデータを出力
                data_hex = bytes.fromhex(f"43 61 06 99 {deg_value:02x}")
                crc = SenderFrame.crc8(data_hex)
                self.serial_instance.write(data_hex + bytes([crc]))
            else:
                # 範囲外の場合は出力しない
                pass
        except ValueError:
            # 入力が整数でない場合は何もしない
            pass

    def send_data(self, data_to_send):
        if data_to_send == "flight":
            data_hex = bytes.fromhex("43 61 05 00 4E")
        elif data_to_send == "sleep":
            data_hex = bytes.fromhex("43 61 05 01 49")
        elif data_to_send == "elace":
            data_hex = bytes.fromhex("43 61 05 05 55")
        elif data_to_send == "reboot":
            data_hex = bytes.fromhex("43 61 05 77 0C")
        else:
            #print(f"Invalid input: {data_to_send}")
            return

        self.serial_instance.write(data_hex)


class ReceiverFrame(tk.Frame):
    saveFileName = datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".log"
    File = open(saveFileName, "w")

    def __init__(self, master=None, serial_instance=None):
        super().__init__(master)
        self.master = master
        self.serial_instance = serial_instance
        self.pack()

        self.create_widgets()
        self.receive_data()

    def create_widgets(self):

        self.received_data_listbox = tk.Listbox(self, width=80, height=10)
        self.received_data_listbox.pack(pady=10, padx=10, expand=True, fill=tk.BOTH)

    firstLaunch = True
    def receive_data(self):
        if self.firstLaunch:
            self.received_data_listbox.insert(tk.END, f"Launched")
            self.received_data_listbox.yview(tk.END)
            self.firstLaunch = False
        data = b""
        received = False

        firstByteRecieveTime = 0

        i= 0
        for _ in count():
            if  self.serial_instance.in_waiting > 0:
                byte = self.serial_instance.read(1)
                i += 1
                if i == 1:
                    firstByteRecieveTime = time.time()

                data += byte

                if (i == 14) :
                    received = True
                    break
            
            if (i>0):
                if (time.time() - firstByteRecieveTime > 0.01):
                    received = True
                    break  

            self.update()

        if received:
            hex_data = ', '.join([format(b, '02x') for b in data])
            self.File.write("time,"+"{:.5f}".format(time.time())+","+datetime.datetime.now().strftime("date,%Y,%m,%d,%H,%M,%S,data,") + hex_data + "\n")
            self.File.flush()
            self.received_data_listbox.insert(tk.END, datetime.datetime.now().strftime("%Y%m%d-%H%M%S")+f":  {hex_data}")
            self.received_data_listbox.yview(tk.END)

        self.master.after(50, self.receive_data)


class MainApplication(tk.Tk):
    def __init__(self, config=None):
        super().__init__()
        self.title("C-71J Valve System Ground Control")
        self.config = config if config else ConfigParser()

        if not self.config.has_section("COM"):
            self.config.add_section("COM")
        self.config.set("COM", "port", "COM13")
        self.config.set("COM", "baudrate", "115200")

        com_port = self.config.get("COM", "port")
        baudrate = int(self.config.get("COM", "baudrate"))
        self.serial_instance = serial.Serial(com_port, baudrate, timeout=1)

        self.iconbitmap(default='logo.ico')

        self.create_widgets()

    def create_widgets(self):
        tk.Label(self, text=self.config.get("COM", "port")).pack(pady=3)

        tk.Label(self, text="Baudrate: "+self.config.get("COM", "baudrate") + " bps").pack(pady=3)

        self.sender_frame = SenderFrame(self, serial_instance=self.serial_instance)
        self.sender_frame.pack(side="left", padx=10)

        self.receiver_frame = ReceiverFrame(self, serial_instance=self.serial_instance)
        self.receiver_frame.pack(side="right", padx=10, fill=tk.BOTH, expand=True)

if __name__ == "__main__":
    app = MainApplication()
    app.mainloop()
