# Basic Linux Commands Cheat Sheet

## Moaz Hamdy


🔹 **Command: `whoami`**  
**Description:**  
Displays the username of the currently logged-in user.  

**Examples:**  
```bash
# Show the current user
whoami
```

---

🔹 **Command: `man`**  
**Description:**  
Displays the manual for a command.  

**Examples:**  
```bash
# Show the manual for ls command
man ls
```

---

🔹 **Command: `clear`**  
**Description:**  
Clears the terminal screen.  

**Examples:**  
```bash
# Clear the terminal
clear
# Clear screen but keep scrollback history
clear-x
```

---

🔹 **Command: `pwd`**  
**Description:**  
Prints the current working directory (shows where you are in the filesystem).  

**Examples:**  
```bash
# Show the current directory path
pwd
```

---

🔹 **Command: `ls`**  
**Description:**  
Lists files and directories in the current location.  

**Examples:**  
```bash
# List files
ls

# List files with details
ls -l

# List hidden files
ls -a
```

---

🔹 **Command: `cd`**  
**Description:**  
Changes the current directory.  

**Examples:**  
```bash
# Move to Documents folder
cd Documents

# Move to parent directory
cd ..

# Move to home directory
cd ~
```

---

🔹 **Command: `mkdir`**  
**Description:**  
Creates a new directory (folder).  

**Examples:**  
```bash
# Create a folder named test
mkdir test

# Create nested folders
mkdir -p folder1/folder2
```

---

🔹 **Command: `touch`**  
**Description:**  
Creates an empty file or updates the timestamp of an existing file.  

**Examples:**  
```bash
# Create a file named notes.txt
touch notes.txt
```

---

🔹 **Command: `rm`**  
**Description:**  
Removes (deletes) files or directories.  

**Examples:**  
```bash
# Delete a file
rm file.txt

# Delete a folder and its contents
rm -r folder

# Delete a folder interactively (asks for confirmation before each file)
rm -ir myfolder

```

---

🔹 **Command: `open`** (MacOS only)  
**Description:**  
Opens a file, folder, or application using the default program.  

**Examples:**  
```bash
# Open a file
open notes.txt

# Open current folder in Finder
open .
```

---

🔹 **Command: `mv`**  
**Description:**  
Moves or renames files and directories.  

**Examples:**  
```bash
# Rename a file
mv oldname.txt newname.txt

# Move file to another folder
mv file.txt Documents/

# Rename a file and show what was done
mv -v oldname.txt newname.txt

# Move a file into Documents and show the action
mv -v file.txt Documents/

```

---

🔹 **Command: `cp`**  
**Description:**  
Copies files and directories.  

**Examples:**  
```bash
# Copy a file
cp file.txt copy.txt

# Copy folder with contents
cp -r folder1 folder2
```
---
---
