import sys

from pprint import pprint


def do_stuff(file):
    result = {}
    with open(file, "r") as f_in:
        for line in f_in.readlines():
            tkns = line.split()
            lhs = []
            rhs = []
            i = 0
            for tok in tkns:
                if tok is ":" and i == 0:
                    i = 1
                elif i == 0:
                    lhs.append(tok)
                else:
                    rhs.append(tok)
            if len(rhs) == 0:
                continue
            result[tuple(lhs)] = rhs
    return result


def main(f):
    d = do_stuff(f)
    pprint(d)


if __name__ == '__main__':
    try:    
        main(sys.argv[1])
    except IndexError as err:
        print("No input file given")
