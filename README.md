qbmove_firmware
===============

### IMPORTANT: Versioning:
From now on, each of the repositories in this table will have a version number composed
by 3 numbers in this form **v x.y.z**
The current version is a #define in globals.h

|  Tools          |  Libraries |  Firmware                |
|-----------------|------------|--------------------------|
| qbmove simulink | qbAPI      | qbmove firmware          |
| qbmoveadmin     |            | qbmove advanced firmware |
| handmoveadmin   |            | hand firmware micro      |

I will use **x, y, z** followed by **T, L, F** when talking about respectively Tools, Libraries or Firmware repositories.
E.g. **xT** will be the the number **x** of the Tools whereas **yF** will be the **y** number of the Firmware version.

- Every change in the number **z** means a changement in the respective repo which not implies changements in other repos.
- Every change in the number **y** in a repo is backward compatibile reading the table from right to left. This means that
if you have some new feature in a Firmware, you can still use old Libraries and Tools for management, but of coruse
you will not be able to use the new features. In this case the rule is **yF >= yL >= yT**.
- Every change in the number **x** means a changement which is not backward compatibile, hence you will need to update
everything to use it. In this case the rule is **xF = xL = xT**.

Summarising
- **z** independent
- **yF >= yL >= yT**
- **xF = xL = xT**

E.g.

| Tools              | Libraries    | Firmware               | Compatible |
|--------------------|--------------|------------------------|------------|
| qbmoveadmin v4.2.3 | qbAPI v4.5.0 | qbmove firmware v4.6.7 | YES        |
| qbmoveadmin v3.2.3 | qbAPI v4.5.0 | qbmove firmware v4.6.7 | NO         |
| qbmoveadmin v4.2.3 | qbAPI v4.5.7 | qbmove firmware v4.5.0 | YES        |

Firmware for qbmove for *qbcontrol_beta4.2* board

Just open it with PSoC Creator and upload it onto the board
