import smbus
from time import sleep

class LCD:
    # Default I2C address is usually 0x27 or 0x3f
    def __init__(self, addr=0x27, port=1):
        self.addr = addr
        self.bus = smbus.SMBus(port)
        self.define_commands()
        self.lcd_init()
        self.colors = ["", "", "", ""]

    def define_commands(self):
        self.LCD_CHR = 1 # Mode - Sending data
        self.LCD_CMD = 0 # Mode - Sending command
        self.LINE_1 = 0x80 # LCD RAM address for the 1st line
        self.LINE_2 = 0xC0 # LCD RAM address for the 2nd line
        self.BACKLIGHT = 0x08  # On: 0x08, Off: 0x00
        self.ENABLE = 0b00000100 # Enable bit

    def lcd_init(self):
        self.lcd_byte(0x33, self.LCD_CMD) # 110011 Initialise
        self.lcd_byte(0x32, self.LCD_CMD) # 110010 Initialise
        self.lcd_byte(0x06, self.LCD_CMD) # 000110 Cursor move direction
        self.lcd_byte(0x0C, self.LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
        self.lcd_byte(0x28, self.LCD_CMD) # 101000 Data length, number of lines, font size
        self.lcd_byte(0x01, self.LCD_CMD) # 000001 Clear display
        sleep(0.0005)

    def lcd_byte(self, bits, mode):
        # High bits
        bits_high = mode | (bits & 0xF0) | self.BACKLIGHT
        # Low bits
        bits_low = mode | ((bits << 4) & 0xF0) | self.BACKLIGHT

        self.bus.write_byte(self.addr, bits_high)
        self.lcd_toggle_enable(bits_high)
        self.bus.write_byte(self.addr, bits_low)
        self.lcd_toggle_enable(bits_low)

    def lcd_toggle_enable(self, bits):
        sleep(0.0005)
        self.bus.write_byte(self.addr, (bits | self.ENABLE))
        sleep(0.0005)
        self.bus.write_byte(self.addr, (bits & ~self.ENABLE))
        sleep(0.0005)

    def display_string(self, message, line):
        self.lcd_byte(line, self.LCD_CMD)
        for i in range(16):
            self.lcd_byte(ord(message[i]), self.LCD_CHR)

    def clear(self):
        self.lcd_byte(0x01, self.LCD_CMD)
        
        
