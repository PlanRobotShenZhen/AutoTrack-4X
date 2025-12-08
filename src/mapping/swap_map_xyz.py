input_file = "finalCloud.pcd"
output_file = "finalCloud_swapped.pcd"

with open(input_file, "r") as fin, open(output_file, "w") as fout:
    for line in fin:
        if line.strip() == "" or (line.strip()[0] not in "-0123456789"):
            fout.write(line)
            continue
        parts = line.strip().split()
        if len(parts) >= 3:
            # swap y and z
            parts[1], parts[2] = parts[2], parts[1]
            # swap x and y
            parts[0], parts[1] = parts[1], parts[0]
            fout.write(" ".join(parts) + "\n")
        else:
            fout.write(line)