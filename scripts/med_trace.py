#!env python3
# sudo pip3 install pyserial cobs
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

sMedTraceFormat = "{},"*14 + "{}\n" 

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
        if len(arr) < 25:
            sys.stderr.write("Unexpected packet {}".format(str(binascii.hexlify(arr))))
            return
        
        idx = 0
        seq       = arr[idx]; idx += 1
        bitmap    = arr[idx]; idx += 1
        hallState = arr[idx]; idx += 1
        elAngle  = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Ia       = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Ib       = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Iq       = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Id       = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Valph    = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Vbeta    = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        elSpeed  = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        target = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Iqref    = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2
        Idref    = raw2int16(arr[idx] + arr[idx+1] * 256); idx += 2

        state = bitmap >> 2
        direction = bitmap & 1
        #print("Decoded SN {}".format(seq))

        if self.seq is not None and seq != ((self.seq + 1) & 0xFF):
            sys.stderr.write("-{}".format(seq))
        self.seq = seq
        self.outf.write(sMedTraceFormat.format(
            seq, state, direction, hallState, elAngle,
            Ia, Ib, Iq, Id, Valph, Vbeta,
            elSpeed, target, Iqref, Idref))
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
        with open(outfn, 'w') as outf, serial.Serial(dev, 3000000, timeout=10) as ser:
            outf.write('seq, state, direction, hallState, elAngle, Ia, Ib, Iq, Id, Valph, Vbeta, elSpeed, target, Iqref, Idref\n')
            med = MedTrace(outf, Nsample)
            while med.process(ser.read(256)): # try to batch, for efficiency
                if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []): # want to bail?
                    #c = sys.stdin.read(1)
                    #if c == 'q': break
                    break # don't care what the key was
        # ser.close()
        return

    # Plot
    # read the data file back again; faster than building up the data file 1 row at a time
    seq, state, direction, hallState, elAngle, Ia, Ib, Iq, Id, Valph, \
        Vbeta, elSpeed, target, Iqref, Idref = \
            np.loadtxt(outfn, dtype=np.int16, unpack=True, delimiter = ',',
            skiprows=1, max_rows=Nsample)

    # Convert to easy-to-read unit
    eAngle = elAngle * 180.0 / 0x8000
    eSpeed = elSpeed * 0.1 # * 180 / np.pi
    #mSpeed = meSpeed * 0.1 # * 180 / np.pi
    target = target * 0.1
    Vdd = 3.3; Rshunt = 0.01; Gaop = 5.18
    Imax = Vdd / (2 * Rshunt * Gaop)
    Vmax = 3.3 * 19.17 # voltage divider gain is 19.17
    Ia = Ia * Imax / 0x8000; Ib = Ib * Imax / 0x8000
    Id = Id * Imax / 0x8000; Iq = Iq * Imax / 0x8000
    Idref = Idref * Imax / 0x8000; Iqref = Iqref * Imax / 0x8000
    Valph = Valph * Vmax / 0x8000; Vbeta = Vbeta * Vmax / 0x8000

    t = 0.001 * np.linspace(0, len(seq), num=len(seq))

    fig = plt.figure(figsize=(12, 12)) # width, height size
    Nrow = 3
    yy1 = plt.subplot(Nrow,1,1)
    yy1.plot(t, seq, '.', marker='.', markersize=1, label='SN')
    yy1.plot(t, state, label='MC state')
    yy1.plot(t, direction, ls='-.', label='direction')
    #yy1.scatter(t, state, label='state')
    yy1.plot(t, eAngle, label='E ∠ [°]')
    yy1.legend(bbox_to_anchor=(0, 1.02, 1., .102), ncol=5, mode="expand", borderaxespad=0.5)
    yy1.set(ylabel='[°]')

    yy2 = yy1.twinx()
    yy2.scatter(t, hallState, label='Hall')
    yy2.plot(t, eSpeed, label='Espeed [rad/s]')
    yy2.plot(t, target, 'k', label='target [rad/s]')
    yy2.set(ylabel='[1/s]')
    yy2.legend()

    #yy3 = yy1.twinx()

    yy1 = plt.subplot(Nrow,1,2)
    yy1.plot(t, Ia, ls=':', label='Ia')
    yy1.plot(t, Ib, ls=':', label='Ib')
    yy1.plot(t, Id, label='Id')
    yy1.plot(t, Iq, label='Iq')
    yy1.plot(t, Idref, ls='-.', label='Id*')
    yy1.plot(t, Iqref, ls='-.', label='Iq*')
    yy1.set(ylabel='[A]')
    yy1.legend(bbox_to_anchor=(0, 1.02, 1., .102), ncol=6, mode="expand", borderaxespad=0.5)

    yy2 = yy1.twinx()
    yy2.plot(t, Valph, 'b', ls='--', label=r'$V_\alpha$')
    yy2.plot(t, Vbeta, 'g', ls='--', label=r'$V_\beta$')
    yy2.set(ylabel='[V]')
    yy2.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main(sys.argv[1:])
