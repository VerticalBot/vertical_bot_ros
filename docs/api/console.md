# The JavaScript console

```{image} ../_static/screens/19-console.png
:alt: Console tab
:class: screenshot
```

The **Console (RoboDK API)** tab runs JavaScript against the open station with the same API as the Python
package: `RDK` is the `Robolink` instance, `ITEM_TYPE_*` constants and `Mat` helpers are in scope.

```javascript
const r = RDK.Item('', ITEM_TYPE_ROBOT);      // first robot
const p = RDK.AddProgram('API demo', r);
p.MoveJ(r.Joints());
const t = RDK.AddTarget('T1', r.getLink(ITEM_TYPE_FRAME), r);
t.setPose(transl(500, 0, 400) * rotx(Math.PI));
p.MoveL(t);
print(p.Update());                             // [valid, duration, distance, ...]
```

`print()` writes to the console; results of the last expression are shown; errors are reported inline.
The history is kept per browser. Everything executed is undoable.

Use the console to prototype scripts before moving them to Python (identical calls) or to build
{doc}`plugins`.
