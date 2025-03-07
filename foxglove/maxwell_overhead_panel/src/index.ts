import { ExtensionContext } from "@foxglove/extension";

import { initOverhead2DPanel } from "./Overhead2D";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Maxwell Overhead", initPanel: initOverhead2DPanel });
}
