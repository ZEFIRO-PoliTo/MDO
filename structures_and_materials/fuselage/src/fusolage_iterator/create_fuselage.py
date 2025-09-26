import sys

def create_fuselage(length, width, height, filename="fuselage.vsp3"):
    # TODO: Implement OpenVSP fuselage creation
    pass

def main():
    if len(sys.argv) < 4:
        print("Usage: python create_fuselage.py <length> <width> <height> [output.vsp3]")
        sys.exit(1)
    length = float(sys.argv[1])
    width = float(sys.argv[2])
    height = float(sys.argv[3])
    output = sys.argv[4] if len(sys.argv) > 4 else "fuselage.vsp3"
    create_fuselage(length, width, height, output)

if __name__ == "__main__":
    main()
