# PyRL

```shell
├── README.md
├── Simulators
│   ├── SUMO
│   │   └── network
│   │       └── Anl427  <- SUMO network files. Should be moved to src ->
│   │           ├── anl427.add.xml
│   │           ├── anl427.net.xml
│   │           ├── anl427.rou.alt.xml
│   │           ├── anl427.rou.xml
│   │           ├── anl427.rou.xml.bak
│   │           ├── anl427.sumocfg
│   │           ├── anl427.trips.xml
│   │           ├── detector2routeid.txt
│   │           ├── detectorroute.py
│   │           └── trips.trips.xml
│   └── simulators.md
├── dockerfile  <- TODO: set up simulator in docker for ease of use / portability ->
├── links.md
├── report
│   ├── include
│   └── tex
│       ├── chapters
│       │   ├── instance.tex
│       │   └── introduction.tex
│       ├── appendices
│       ├── bib
│       │   └── Bibliography.bib
│       ├── report.pdf
│       └── report.tex
└── src
    ├── data
    │   ├── anl427.add.xml
    │   ├── anl427.net.xml
    │   ├── anl427.rou.xml
    │   └── anl427.sumocfg
    ├── preprocessing  <- Some scripts used to make sense of the data given by swarco ->
    │   ├── README.md  
    │   ├── TLMovement.py
    │   ├── detector2routeid.txt
    │   ├── detectorroute.py
    │   └── mastraparse.py
    ├── runner.py  <- This is the main python file, which will set up the simulation and run the algorithm(s) ->
    └── tripinfo.xml
```