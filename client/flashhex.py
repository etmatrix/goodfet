#!/usr/bin/env python

import sys;
import binascii;
from time import sleep

from GoodFETMSP430 import GoodFETMSP430;
from intelhex import IntelHex16bit, IntelHex;


#Initialize FET and set baud rate
client=GoodFETMSP430();
client.serInit()

client.setup();
client.start();

print "Identifies as %s (%04x)" % ( client.MSP430identstr(), client.MSP430ident());

client.erase();

sleep(3);

f=sys.argv[1];

h = IntelHex16bit(f);

skip = 0;

for i in h._buf.keys():
    if skip == 1:
        skip = 0;
        continue;
    else:
        skip = 1;
    print "Hex: %04x %04x" % (i, h[i>>1]);
    for a in range(1,5):
        client.MSP430pokeflash(i, h[i>>1]);

#        sleep(0.10);
        data = client.MSP430peek(i);
        print "Read: %04x %04x" % (i, data);
        if data!=h[i>>1]:
            print "retry";
        else:
            break;

client.MSP430releasecpu();
client.MSP430stop();
