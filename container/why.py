with open("container.ino", "r") as f:
    data = f.readlines()

out = ""
catch = ["void", "int", "float", "String"]
for line in data:
    first = line.split(" ")[0]
    if first in catch:
        definer = True
    else:
        definer = False

    linestart = True
    currLine = ""
    for char in line:
        if linestart:
            if char != " ":
                linestart = False
                currLine += char
        else:
            currLine += char

    if definer:
        out += "\n"
    
    out += currLine

print(out)

while True:
    pass