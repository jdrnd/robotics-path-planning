# Reads a PGM image file into a list of lists containing 0-255 values
def read_pgm(filename):
    with open(filename, 'rb') as pgmf:
        header = pgmf.readline() # reads pgm magic number

        if header != b'P5\n':
            return []

        # width and height, skip comment lines

        while True:
            l = pgmf.readline()
            if l[0] != ord(b'#'):
                break

        (width, height) = [int(i) for i in l.split()]
        depth = pgmf.readline()
        data = []
        for _ in range(height):
            row = []
            for _ in range(width):
                row.append(ord(pgmf.read(1))) # each file byte is one pixel
            data.append(row)

    return data
