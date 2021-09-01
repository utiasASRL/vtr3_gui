/**
 * Copyright 2021, Autonomous Space Robotics Lab (ASRL)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

import "leaflet-rotatedmarker";
import React from "react";
import { Marker as LeafletMarker } from "leaflet";
import { LeafletProvider, withLeaflet, MapLayer } from "react-leaflet";

class RotatedMarker extends MapLayer {
  static defaultProps = {
    rotationOrigin: "center",
  };

  createLeafletElement(props) {
    const el = new LeafletMarker(props.position, this.getOptions(props));
    this.contextValue = { ...props.leaflet, popupContainer: el };
    return el;
  }

  updateLeafletElement(fromProps, toProps) {
    if (toProps.position !== fromProps.position)
      this.leafletElement.setLatLng(toProps.position);
    if (toProps.icon !== fromProps.icon)
      this.leafletElement.setIcon(toProps.icon);
    if (toProps.zIndexOffset !== fromProps.zIndexOffset)
      this.leafletElement.setZIndexOffset(toProps.zIndexOffset);
    if (toProps.opacity !== fromProps.opacity)
      this.leafletElement.setOpacity(toProps.opacity);
    if (toProps.draggable !== fromProps.draggable) {
      if (toProps.draggable === true) this.leafletElement.dragging.enable();
      else this.leafletElement.dragging.disable();
    }
    if (toProps.rotationAngle !== fromProps.rotationAngle)
      this.leafletElement.setRotationAngle(toProps.rotationAngle);
    if (toProps.rotationOrigin !== fromProps.rotationOrigin)
      this.leafletElement.setRotationOrigin(toProps.rotationOrigin);
  }

  render() {
    const { children } = this.props;
    return children == null || this.contextValue == null ? null : (
      <LeafletProvider value={this.contextValue}>{children}</LeafletProvider>
    );
  }
}

export default withLeaflet(RotatedMarker);
