import serial
import random
import argparse
import time

def test_prep_bin_packet():
    """
    Unit test for the prep_bin_packet function.

    This test verifies that the prep_bin_packet function correctly formats
    binary data into the expected packet structure. It can be run to ensure
    the function behaves as expected, especially after any modifications.

    The test checks:
    1. The correct formatting of a packet with a known input.
    2. The correct handling of random input data of varying lengths.

    No parameters are required as it generates its own test data.

    Returns:
    None. Assertions will raise exceptions if tests fail.
    """    # Test with 2 bytes of binary data
    test_data = bytes([0xAA, 0xBB])
    result = prep_bin_packet(test_data)
    
    # Expected output:
    # Ts000002AABB\n
    expected = bytes("Ts000002aabb\n", 'utf-8')
    
    assert result == expected, f"Expected {expected}, but got {result}"

def test_prep_bin_packet_random():
    # Test with random length (1 to 1000 bytes) of binary data
    length = random.randint(1, 1000)
    test_data = bytes([random.randint(0, 255) for _ in range(length)])
    result = prep_bin_packet(test_data)
    
    # Expected output format:
    # Ts[6-digit length][hex data]\n
    expected_prefix = f"Ts{length:06d}"
    expected_suffix = "\n"
    expected_hex = test_data.hex()
    
    assert result.startswith(bytes(expected_prefix, 'utf-8')), f"Expected to start with {expected_prefix}, but got {result[:8]}"
    assert result.endswith(bytes(expected_suffix, 'utf-8')), f"Expected to end with {expected_suffix}, but got {result[-1:]}"
    assert result[8:-1].decode('utf-8') == expected_hex, f"Expected hex data {expected_hex}, but got {result[8:-1].decode('utf-8')}"
    
    print(f"Random test passed with {length} bytes of data")

def prep_bin_packet(bindata):
    """
    Prepare a binary packet for transmission according to the specified protocol.

    The protocol format is as follows:
    1. Preamble: "Ts" (2 bytes)
    2. Payload length: 6 bytes, zero-padded string representation of the length
    3. Payload: Hexadecimal representation of the binary data
    4. Terminator: "\n" (1 byte)

    Examples:
    1. For bindata = b'\xAA\xBB':
       Output: b'Ts000002aabb\n'
    
    2. For bindata = b'\x01\x23\x45\x67\x89\xAB\xCD\xEF':
       Output: b'Ts0000080123456789abcdef\n'

    Args:
        bindata (bytes): The binary data to be packaged.

    Returns:
        bytes: The formatted packet ready for transmission.
    """
    hex_payload = bindata.hex()
    packet = bytes("Ts" + "{:06d}".format(len(bindata)) + hex_payload + "\n", 'utf-8')
    print(packet)
    return packet

def send_bin_packet(teensy_ser, bindata):
    """
    Send a binary packet to the Teensy.

    This function prepares a binary packet using the prep_bin_packet function,
    sends it to the Teensy, and waits for an acknowledgment. If an error occurs,
    it will recursively retry the packet send.

    Args:
        teensy_ser (serial.Serial): The serial connection to the Teensy.
        bindata (bytes): The binary data to be sent.

    Raises:
        ValueError: If there's an error in the packet that can't be resolved by resending.

    Note:
        The function will print the response from the Teensy for debugging purposes.
    """
    prepped_pkt = prep_bin_packet(bindata)
    teensy_ser.write(prepped_pkt)

    while b'OK' not in (rsp := teensy_ser.readline()):
        print(rsp)
        # Find Error: in rsp and abort
        if b'Error' in rsp:
            if b'Invalid packet format' in rsp:
                # Try again
                print('---- Dropped, trying again ----')
                send_bin_packet(teensy_ser, bindata)
                return
            else:
                raise ValueError(f"Error in packet: {rsp}")
    print(rsp)

def crc32(data):
    """
    Calculate the CRC-32 checksum of the given data.

    Args:
        data (bytes or bytearray): The input data for which to calculate the CRC-32.

    Returns:
        int: The calculated CRC-32 checksum.

    Note:
        This function uses the standard CRC-32 polynomial (0xEDB88320) 
        and initial value (0xFFFFFFFF).
    """
    crc = 0xFFFFFFFF
    for byte in data:
        if isinstance(data, bytearray):
            byte = int(byte)
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc = crc >> 1
    return ~crc & 0xFFFFFFFF

def program_cortex(teensy_port="COM13", scum_port="COM18", binary_image="./code.bin",
        boot_mode='optical', skip_reset=False, insert_CRC=False,
        pad_random_payload=False):
    """
    Inputs:
        teensy_port: String. Name of the COM port that the Teensy
            is connected to.
        scum_port: String. Name of the COM port that the UART 
            is connected to. If None, does not attempt to connect
            to SCM via UART.
        binary_image: String. Path to the binary file to 
            feed to Teensy to program SCM. This binary file shold be
            compiled using whatever software is meant to end up 
            on the Cortex. This group tends to compile it using Keil
            projects.
        boot_mode: String. 'optical' or '3wb'. The former will assume an
            optical bootload, whereas the latter will use the 3-wire
            bus.
        skip_reset: Boolean. True: Skip hard reset before optical 
            programming. False: Perform hard reset before optical programming.
        insert_CRC: Boolean. True = insert CRC for payload integrity 
            checking. False = do not insert CRC. Note that SCM C code 
            must also be set up for CRC check for this to work.
        pad_random_payload: Boolean. True = pad unused payload space with 
            random data and check it with CRC. False = pad with zeros, do 
            not check integrity of padding. This is useful to check for 
            programming errors over full 64kB payload.
    Outputs:
        No return value. Feeds the input from binary_image to the Teensy to program SCM
        and programs SCM. 
    Raises:
        ValueError if the boot_mode isn't 'optical' or '3wb'.
    Notes:
        When setting optical parameters, all values can be toggled to improve
        success when programming. In particular, too small a third value can
        cause the optical programmer to lose/eat the short pulses.
    """
    # Open COM port to Teensy
    teensy_ser = serial.Serial(
        port=teensy_port,
        baudrate=10000000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=5,
        write_timeout=5
    )

    # Open binary file from Keil
    with open(binary_image, 'rb') as f:
        bindata = bytearray(f.read())

    # Need to know how long the binary payload is for computing CRC
    code_length = len(bindata) - 1
    pad_length = 65536 - code_length - 1

    # Optional: pad out payload with random data if desired
    # Otherwise pad out with zeros - uC must receive full 64kB
    if(pad_random_payload):
        for i in range(pad_length):
            bindata.append(random.randint(0,255))
        code_length = len(bindata) - 1 - 8
    else:
        for i in range(pad_length):
            bindata.append(0)

    if insert_CRC:
        # Insert code length at address 0x0000FFF8 for CRC calculation
        # Teensy will use this length value for calculating CRC
        bindata[65528] = code_length % 256 
        bindata[65529] = code_length // 256
        bindata[65530] = 0
        bindata[65531] = 0
    
    # Transfer payload to Teensy
    teensy_ser.write(b'transfersram\n')
    print(teensy_ser.readline())
    teensy_ser.write(b'\r\n')
    print(teensy_ser.readline())
    time.sleep(0.1)
    # We seem to have a limit of 64 total packet length
    chunk_size = 16  # Each byte is represented by 2 hex characters
                     # chunk_size = 16 results in a packet of 41
    for i in range(0, len(bindata), chunk_size):
        chunk = bindata[i:i+chunk_size]
        send_bin_packet(teensy_ser, chunk)

    # Wait for the final confirmation
    resp = teensy_ser.readline()
    while(len(resp) == 0):
        print("Waiting for reply...")
        resp = teensy_ser.readline()

    print(resp)
    if(resp != b'SRAM Transfer Complete\r\n'):
        raise ValueError("Teensy did not report ready after sending data")

    time.sleep(1)

    resp = teensy_ser.readline()
    # Parse the CRC out of b'CRC: 0x[32-bit hex]
    crc_str = resp.split(b'CRC: ')[1].split(b'\r\n')[0].decode('utf-8')
    crc_val = int(crc_str, 16)

    # Calculate crc locally on bindata and compare with Teensy's
    crc_local = crc32(bindata)
    if crc_local != crc_val:
        raise ValueError("CRC mismatch between Teensy and local calculation")
    
    print(f"CRC matched: Teensy: 0x{crc_val:08X}, PC: 0x{crc_local:08X}")

    if insert_CRC:
        # Have Teensy calculate 32-bit CRC over the code length 
        # It will store the 32-bit result at address 0x0000FFFC
        teensy_ser.write(b'insertcrc\n')

    if boot_mode == 'optical':
        # Configure parameters for optical TX
        teensy_ser.write(b'configopt\n')
        teensy_ser.write(b'80\n')
        teensy_ser.write(b'80\n')
        teensy_ser.write(b'3\n')
        teensy_ser.write(b'80\n')
        
        # Encode the payload into 4B5B for optical transmission
        teensy_ser.write(b'encode4b5b\n')

        if not skip_reset:
            # Do a hard reset and then optically boot
            teensy_ser.write(b'bootopt4b5b\n')
        else:
            # Skip the hard reset before booting
            teensy_ser.write(b'bootopt4b5bnorst\n')

        # Display confirmation message from Teensy
        print((rsp := teensy_ser.readline()))
        if(rsp != b'Optical Boot Complete\r\n'):
            raise ValueError("Optical Boot failed: Teensy did not report 'Optical Boot Complete'")
        # delay 2 seconds in case image takes long to complete CRC check for large size.
        time.sleep(2)
        teensy_ser.write(b'opti_cal\n');
    elif boot_mode == '3wb':
        # Execute 3-wire bus bootloader on Teensy
        teensy_ser.write(b'boot3wb\n')

        # Display confirmation message from Teensy
        print(teensy_ser.readline())
        print(teensy_ser.readline())
        teensy_ser.write(b'3wb_cal\n')
    else:
        raise ValueError("Boot mode '{}' invalid.".format(boot_mode))

    teensy_ser.close()

    # Open UART connection to SCM
    if scum_port != None:
        uart_ser = serial.Serial(
            port=scum_port,
            baudrate=19200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=.5)

        # After programming, several lines are sent from SCM over UART
        for _ in range(5):
            print(uart_ser.readline())

        uart_ser.close()

    return

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='bootlaod script arguments')
    
    parser.add_argument('-tp', '--teensy_port',
        dest='teensy_port', 
        default='COM13',
        action='store', 
        help='Name of the COM port that the Teensy\
            is connected to.'
    )
    
    parser.add_argument('-sp', '--scum_port',
        dest='scum_port',
        action='store',
        default=None,
        help='Name of the COM port that the UART \
            is connected to. If None, does not attempt to connect\
            to SCM via UART.'
    )
    
    parser.add_argument('-i','--image',
        dest='binary_image',
        default="C:\\Projects\\Repositories\\scum-test-code\\scm_v3c\\applications\\log_ads\\Objects\\log_ads.bin",
        help='Path to the binary file to \
            feed to Teensy to program SCM. This binary file shold be\
            compiled using whatever software is meant to end up \
            on the Cortex. This group tends to compile it using Keil\
            projects.'
    )
    
    parser.add_argument('-bm','--boot_mode',
        dest='boot_mode',
        default='optical',
        help="'optical' or '3wb'. \
            The former will assume an optical bootload,\
            whereas the latter will use the 3-wire \
            bus."
    )
    
    parser.add_argument('-sr','--skip_reset',
        dest='skip_reset',
        default=False,
        help='Boolean. True: Skip hard reset before optical \
            programming. False: Perform hard reset before \
            optical programming.'
    )
    
    parser.add_argument('-c','--insert_CRC',
        dest='insert_CRC',
        default=True,
        help='True = insert CRC for payload integrity \
            checking. False = do not insert CRC. Note that SCM C code \
            must also be set up for CRC check for this to work.'
    )
    
    parser.add_argument('-pl','--pad_random_payload',
        dest='pad_random_payload',
        default=False,
        help='True = True = pad unused payload space with \
            random data and check it with CRC. False = pad with zeros, do \
            not check integrity of padding. This is useful to check for \
            programming errors over full 64kB payload.'
    )
    
    argspace = vars(parser.parse_args())
    program_cortex(**argspace)