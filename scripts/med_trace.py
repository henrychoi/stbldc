#!env python3

import sys
import serial
import select
import tty
import binascii
from cobs import cobs
import numpy as np
import matplotlib.pyplot as plt

# @brief many of the ST MC SDK variables are int16, but I transfer
# everything as just byte array, so the sign conversion routine is important
def raw2int16(n): return n if n < 0x8000 else n - 0x10000
def int32(n): return n if n < 0x80000000 else n - 0x100000000

sMedTraceFormat = "{},"*15 + "{}\n" 

class MedTrace:
    def __init__(self, outf, Nsample):
        self.buf = bytearray()
        self.synced = False
        self.outf = outf
        self.Nsample = Nsample
        self.seq = None
        #self.data = np.ndarray(shape=(min(Nsample, 10*1000), 16), dtype=np.raw2int16)
        self.row = 0

    # @return void
    def parse(self, arr):
        if len(arr) < 27:
            sys.stderr.write("Unexpected packet {}".format(str(binascii.hexlify(arr))))
            return
        
        idx = 0
        seq       = arr[idx]; idx += 1
        bitmap    = arr[idx]; idx += 1
        hallState = arr[idx]; idx += 1
        elAngle  = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        elSpeed  = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Ia       = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Ib       = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Iq       = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Id       = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Valph    = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Vbeta    = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        meSpeed  = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        meTarget = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Iqref    = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Idref    = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2

        state = bitmap >> 2
        direction = bitmap & 1
        #print("Decoded SN {}".format(seq))

        if self.seq is not None and seq != ((self.seq + 1) & 0xFF):
            sys.stderr.write("-{}".format(seq))
        self.seq = seq
        self.outf.write(sMedTraceFormat.format(
            seq, state, direction, hallState, elAngle, elSpeed,
            Ia, Ib, Iq, Id, Valph, Vbeta,
            meSpeed, meTarget, Iqref, Idref))
        self.row += 1

    # @param s COBS encoded byte array
    # @return Whether collected requested number of sample
    def process(self, s):
        #print("Received {}".format(str(binascii.hexlify(bytearray(s)) )))
        for b in s:
            if self.synced: # copy out the bytes until 0x00
                if b == 0: # frame complete; decode what I've accumulated
                    try:
                        decoded = cobs.decode(self.buf)
                        #print("Decoded {}".format(str(binascii.hexlify(decoded))))
                        self.parse(decoded)
                    except cobs.DecodeError:
                        sys.stderr.write("Decoding error on {}\n".format(str(binascii.hexlify(self.buf))))

                    self.buf.clear() #self.buf = bytearray()
                    #self.Nsample = self.Nsample - 1
                else:
                    self.buf.append(b)
            elif b == 0: # look for the starting 0x00
                self.buf.clear() #self.buf = bytearray()
                self.synced = True
        return self.row < self.Nsample # whether collected enough samples

def main(argv):
    dev = argv[0]
    outfn = argv[1] if (len(argv) > 1) else '/dev/stdout'
    Nsample = int(argv[2]) if (len(argv) > 2) else 0

    if (dev.startswith('/dev/')):
        print("Listening on device = {}".format(dev))
        tty.setcbreak(sys.stdin.fileno()) # make the stdin nonblocking
        #ser = open(dev)
        #ser.reset_input_buffer()
        with open(outfn, 'w', Nsample) as outf, serial.Serial(dev, 3000000, timeout=10) as ser:
            outf.write('seq, state, direction, hallState, elAngle, elSpeed, Ia, Ib, Iq, Id, Valph, Vbeta, meSpeed, meTarget, Iqref, Idref\n')
            med = MedTrace(outf, Nsample)
            while med.process(ser.read(256)): # try to batch, for efficiency
                if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []): # want to bail?
                    #c = sys.stdin.read(1)
                    #if c == 'q': break
                    break # don't care what the key was
        # ser.close()

    # Plot
    # read the data file back again; faster than building up the data file 1 row at a time
    seq, state, direction, hallState, elAngle, elSpeed, \
        Ia, Ib, Iq, Id, Valph, \
            Vbeta, meSpeed, meTarget, Iqref, Idref = \
                np.loadtxt(outfn, dtype=np.int16, unpack=True, max_rows=Nsample, delimiter = ',')

    # Convert to easy-to-see unit
    eAngle = elAngle * 180.0 / 0x10000
    eSpeed = elSpeed * 0.1 # * 180 / np.pi
    mSpeed = meSpeed * 0.1 # * 180 / np.pi

    t = 0.001 * np.linspace(0, len(seq), num=len(seq))

    fig = plt.figure(figsize=(12, 12)) # width, height size
    Nrow = 3
    yy1 = plt.subplot(Nrow,1,1)
    yy1.plot(t, seq, ls=':', label='SN')
    #yy1.scatter(t, state, label='state')
    yy1.plot(t, eAngle, label='E ∠')
    yy1.legend(bbox_to_anchor=(0, 1.02, 1., .102), #left, bottom, width, height
        ncol=4, mode="expand", borderaxespad=0.5)
    #yy1.set(ylabel='Motor State')

    yy2 = yy1.twinx()
    yy2.plot(t, direction, ls='-.', label='direction')
    yy2.scatter(t, hallState, label='Hall')
    yy2.plot(t, eSpeed, label='Espeed')
    yy2.plot(t, mSpeed, label='Mspeed')
    #yy2.plot(t, hallState, label='Hall')
    yy2.legend()

    #yy3 = yy1.twinx()

    #yy1 = plt.subplot(Nrow,1,2)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main(sys.argv[1:])
