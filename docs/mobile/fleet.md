# Fleet management

```{image} ../_static/screens/24-orchard-fleet.png
:alt: Orchard harvesting fleet, Fleet tab
:class: screenshot
```

*Mobile & Fleet › Create fleet…* groups mobile robots under a fleet item with a manager that runs on the world
clock:

- **Task allocation** — auction based on travel time, battery, capabilities, priority and deadlines; tasks of
  type transport, harvest, spray, mow, prune, scout, pollinate, weed, charge, goto or custom.
- **Traffic management** — orchard alleys and work rows are reserved as segments; robots brake for others,
  deadlock-free with creep-through; charging is scheduled below the battery threshold.
- **KPIs** (Fleet tab and API): tasks done/failed/pending, throughput per hour, utilisation, distance,
  energy, mean wait time, and agricultural yield (fruit picked, kg, area worked) per robot and overall.

Missions ({doc}`agriculture`) generate tasks automatically; tasks can also be added through the API and the
Fleet tab. Every assignment can be dispatched to real vehicles through {doc}`vda5050`.
