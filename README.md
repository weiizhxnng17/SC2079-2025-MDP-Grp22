# SC2079-2025-MDP-Grp22

# SC2079-2025-MDP-Grp22

This repository contains the **complete source code for the Final Submission** of the **SC2079: Multidisciplinary Design Project (MDP)** for Academic Year 2025.

---

## 📂 Project Structure

The repository is organized into modular folders representing each system component:

| Folder | Description |
|---------|-------------|
| **stm/** | Embedded firmware for the STM microcontroller controlling robot movement and sensors. |
| **algo/** | Core algorithm logic, including path-planning, exploration, shortest path, and task execution modules. |
| **algo/simulator/** | Frontend simulator built using Next.js and Tailwind CSS for visualizing robot movement and debugging algorithms. |
| **rpi/** | Raspberry Pi interface code handling serial and network communication between STM, PC, and Android tablet. |
| **image_recognition/** | YOLO-based image detection and arrow/marker recognition pipeline used for Task 2. |
| **android/** | Android application used to control the robot remotely and display live updates. |

---

## 🔗 Referenced Repositories

Development of this project referenced and adapted ideas from the following public repositories:

- [pyesonekyaw/CZ3004-SC2079-MDP-Algorithm](https://github.com/pyesonekyaw/CZ3004-SC2079-MDP-Algorithm)  
- [AryanSethi20/MDP-Algorithm](https://github.com/AryanSethi20/MDP-Algorithm)  
- [AlphaeusNg/SC2079-MDP-Group-29](https://github.com/AlphaeusNg/SC2079-MDP-Group-29/tree/main)

These served as useful references for architecture design, communication protocols, and simulator setup.

---

## ⚙️ Setup & Usage

Clone the repository:
```bash
git clone https://github.com/weiizhxnng17/SC2079-2025-MDP-Grp22.git
cd SC2079-2025-MDP-Grp22
