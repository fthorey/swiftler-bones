import subprocess

import raspberrypi

def scan_i2c():

    proc = subprocess.Popen(['sudo', 'i2cdetect', '-y', raspberrypi.i2c_bus_num()],
                            stdout = subprocess.PIPE,
                            close_fds = True)
    std_out_txt, std_err_txt = proc.communicate()

    addr = []
    lines = std_out_txt.rstrip().split("\n")

    if lines[0] in "command not found":
        raise RuntimeError("i2cdetect not found")

    for i in range(0,8):
        for j in range(0,16):
            idx_i = i+1
            idx_j = j*3+4
            cell = lines[idx_i][idx_j:idx_j+2].strip()
            if cell and cell != "--":
                hexAddr = 16*i+j
                if cell == "UU":
                    addr.append([hexAddr, True])
                else:
                    addr.append([hexAddr, False])

    return addr


i2c_addresses = scan_i2c()
print i2c_addresses


