@echo off 


set map="D:\MAPF-instances\warehouse\10x30-w5\10x30-w5"
set output="D:\disjoint-splitting\outputs\warehouse\10x30-w5"
set time=60

for /l %%k in (10,2,20) do (
  for /l %%i in (0,1,49) do (
    echo Agent %%k ; Instance %%i
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH.csv --cutoffTime=%time%
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH-disjoint3.csv -p DISJOINT3 --cutoffTime=%time%
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH-random.csv -p RANDOM --cutoffTime=%time%
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH-singltons.csv -p SINGLETONS --cutoffTime=%time%
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH-width.csv -p WIDTH --cutoffTime=%time%
  )
)


pause

set map="D:\MAPF-instances\10obs-20x20\10obs-20x20"
set output="D:\disjoint-splitting\outputs\10obs-20x20\10obs-20x20"
set time=60

for /l %%k in (30,10,60) do (
  for /l %%i in (0,1,49) do (
    echo Agent %%k ; Instance %%i
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH.csv --cutoffTime=%time%
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH-disjoint3.csv -p DISJOINT3 --cutoffTime=%time%
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH-random.csv -p RANDOM --cutoffTime=%time%
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH-singltons.csv -p SINGLETONS --cutoffTime=%time%
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH-width.csv -p WIDTH --cutoffTime=%time%
  )
)


set map="D:\MAPF-instances\20x20\20x20"
set output="D:\disjoint-splitting\outputs\20x20\20x20"
set time=60

for /l %%k in (20,10,60) do (
  for /l %%i in (0,1,49) do (
    echo Agent %%k ; Instance %%i
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH.csv --cutoffTime=%time%
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH-disjoint3.csv -p DISJOINT3 --cutoffTime=%time%
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH-random.csv -p RANDOM --cutoffTime=%time%
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH-singltons.csv -p SINGLETONS --cutoffTime=%time%
    x64\Release\disjoint-CBS.exe -m %map%.map  -a %map%map-%%kagents-%%i.agents -o %output%-%%kagents-CBSH-width.csv -p WIDTH --cutoffTime=%time%
  )
)

pause
