# Plugins and events

JavaScript plugins extend the studio like RoboDK's plug-in interface, without a compiler:

```javascript
RDK.registerPlugin({
  name: 'Cycle-time badge',
  onEvent(ev) { if (ev.type === 'programUpdated') console.log(ev.program.name, ev.duration); },
  menu: [{ label: 'Export cycle times', action: () => { /* ... */ } }],
  command(name, value) { /* PluginCommand from the API */ },
});
```

Plugins receive station events (`itemAdded`, `itemRemoved`, `selectionChanged`, `poseChanged`,
`programUpdated`, `simulationTick`, `io`, `message`), can add menu entries and dialogs, and answer
`PluginCommand` calls from Python clients. The event queue is also available to API clients through
`EventsListen` / `WaitForEvent` / `EventsLoop`.

Load a plugin with `PluginLoad('name', source)` from the console or a `?plugin=url` query parameter.
