import { Immutable, MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState, useRef } from "react";
import { createRoot } from "react-dom/client";
import { SwerveModulesList } from "./types";
import { drawOnCanvas } from "./canvas";

let currentModulesCommand: SwerveModulesList = {
  front_left: { speed: 0, angle: 0 },
  front_right: { speed: 0, angle: 0 },
  rear_left: { speed: 0, angle: 0 },
  rear_right: { speed: 0, angle: 0 },
};
let currentModulesOdom: SwerveModulesList = {
  front_left: { speed: 0, angle: 0 },
  front_right: { speed: 0, angle: 0 },
  rear_left: { speed: 0, angle: 0 },
  rear_right: { speed: 0, angle: 0 },
};

function Overhead2DPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [messages, setMessages] = useState<undefined | Immutable<MessageEvent[]>>();

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    if (messages) {
      const modulesCommandMsg = messages.find((msg) => msg.topic === "/modules_command")?.message as SwerveModulesList;
      const modulesOdomMsg = messages.find((msg) => msg.topic === "/drive_modules")?.message as SwerveModulesList;
      if (modulesCommandMsg && !modulesOdomMsg) {
        currentModulesCommand = modulesCommandMsg;
      }
      if (modulesOdomMsg && !modulesCommandMsg) {
        currentModulesOdom = modulesOdomMsg;
      }
      drawOnCanvas(canvasRef.current!, currentModulesCommand, currentModulesOdom).catch((err) => console.log(err));
    }
  }, [messages]);

  // We use a layout effect to setup render handling for our panel. We also setup some topic subscriptions.
  useLayoutEffect(() => {
    // The render handler is run by the broader studio system during playback when your panel
    // needs to render because the fields it is watching have changed. How you handle rendering depends on your framework.
    // You can only setup one render handler - usually early on in setting up your panel.
    //
    // Without a render handler your panel will never receive updates.
    //
    // The render handler could be invoked as often as 60hz during playback if fields are changing often.
    context.onRender = (renderState, done) => {
      // render functions receive a _done_ callback. You MUST call this callback to indicate your panel has finished rendering.
      // Your panel will not receive another render callback until _done_ is called from a prior render. If your panel is not done
      // rendering before the next render call, studio shows a notification to the user that your panel is delayed.
      //
      // Set the done callback into a state variable to trigger a re-render.
      setRenderDone(() => done);

      // currentFrame has messages on subscribed topics since the last render call
      setMessages(renderState.currentFrame);
    };

    // After adding a render handler, you must indicate which fields from RenderState will trigger updates.
    // If you do not watch any fields then your panel will never render since the panel context will assume you do not want any updates.

    // tell the panel context that we care about any update to the _topic_ field of RenderState
    context.watch("topics");

    // tell the panel context we want messages for the current frame for topics we've subscribed to
    // This corresponds to the _currentFrame_ field of render state.
    context.watch("currentFrame");

    // subscribe to some topics, you could do this within other effects, based on input fields, etc
    // Once you subscribe to topics, currentFrame will contain message events from those topics (assuming there are messages).
    context.subscribe([{ topic: "/modules_command" }, { topic: "/drive_modules" }]);
  }, [context]);

  // invoke the done callback once the render is complete
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <canvas width={400} height={500} ref={canvasRef} />
  );
}

export function initOverhead2DPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<Overhead2DPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
