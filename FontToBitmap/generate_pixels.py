import sys, os
import fontforge
from PIL import Image

def safe_name(g):
    # try to produce a filename-friendly name for the glyph
    try:
        return g.glyphname
    except Exception:
        return "u%04X" % g.unicode

def main():
    fontfile = "ari-w9500-display.ttf"
    outdir = "TMP/"
    os.makedirs(outdir, exist_ok=True)

    # Optional explicit range (defaults to ASCII 0..127)
    first = 0
    last  = 127

    font = fontforge.open(fontfile)

    for char_size in range(1, 9):
        char_idx_array_str = "" 
        char_array_str = "" 
        char_index = 1
        num_bytes = 1

        for code in range(first, last + 1):
            try:
                # Access glyph by unicode codepoint integer
                g = font[code]
            except Exception:
                # glyph index not present in this font
                char_idx_array_str += "0, "
                continue

            # skip empty glyphs
            try:
                if not g.isWorthOutputting():
                    char_idx_array_str += "0, "
                    continue
            except Exception:
                # fall back: if glyph has no outline and no bitmap, skip
                if (not hasattr(g, "outline") or len(g.outline) == 0) and not getattr(g, "bitmap", False):
                    char_idx_array_str += "0, "
                    continue

            filename = os.path.join(outdir, f"work.png")

            # Export PNG as temporary working file
            try:
                g.export(filename, char_size*8 - 1)
            except Exception as e:
                char_idx_array_str += "0, "
                print(f"Failed to export U+{code:04X}: {e}")
                continue

            # Read back PNG
            img = Image.open(filename).convert("RGB")
            width, height = img.size
            pixels = img.load()

            num_bytes += width*char_size + 1
            char_array_str += str(width)
            for x in range(width):
                for y in range(char_size*8):
                    r, g, b = pixels[x, y]
                    if y % 8 == 0:
                        char_array_str += ", 0b"
                    if r > 0.5:
                        char_array_str += "1"
                    else:
                        char_array_str += "0"
            char_array_str += ",\n\t"
            char_idx_array_str += str(char_index) + ", "
            char_index += (width*char_size) + 1

        char_idx_array_str = char_idx_array_str[:-2]
        char_idx_array_str += "\n};\n"
        char_array_str = char_array_str[:-3]
        char_array_str += "\n};\n"
        with open("output" + str(char_size) + ".txt", "w") as f:
            f.write("const uint16_t ALPHNUM_" + str(char_size) + "_IDX[128] = {\n\t" + char_idx_array_str + "\n")
            f.write("const uint8_t ALPHNUM_" + str(char_size) + "[" + str(num_bytes) + "] = {\n\t0,\n\t" + char_array_str)
        print("Done")

if __name__ == "__main__":
    main()
