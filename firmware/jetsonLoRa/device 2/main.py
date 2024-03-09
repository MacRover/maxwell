from ..lib.sx1262 import SX1262
import time

def cb(events):
    if events & SX1262.RX_DONE:
        msg, err = sx.recv()
        if msg == b'Ping':
            error = SX1262.STATUS[err]
            print('Receive: {}, {}'.format(msg, error))
            sx.send(b'Pong')
    elif events & SX1262.TX_DONE:
        print('TX done.')

# for jetson orin nano, must change cs, irq, rst, and gpio depending on what pins the LoRa chip is connected to.
# CS pin must be connected the NSS pin on the LoRa chip
sx = SX1262(spi_bus=1, clk=13, mosi=37, miso=22, cs=18, irq=20, rst=15, gpio=2) 

# LoRa
sx.begin(freq=923, bw=125.0, sf=11, cr=5, syncWord=0x12,
         power=-5, currentLimit=60.0, preambleLength=8,
         implicit=False, implicitLen=0xFF,
         crcOn=False, txIq=False, rxIq=False,
         tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)

sx.setBlockingCallback(False, cb)
