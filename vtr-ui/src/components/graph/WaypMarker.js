import React from "react";
import {Marker, Tooltip, Popup} from 'react-leaflet';
import L, { icon, LatLng } from "leaflet";
import pinSvg from '../../images/pin-icon.svg'

const pinIcon = new L.Icon({
    iconUrl: pinSvg,
    iconSize: new L.Point(49, 50),
    iconAnchor: [24, 49]
  });
  
class WaypMarker extends React.Component{
    //props it needs: gps, index of this wayp, socket, socketconnected
    constructor(props){
        super(props);
    }

    render(){
        return (
            <Marker
                position={this.props.position}
                id={this.props.id}
                draggable={true}
                ondragend={this._onDragend.bind(this)}
                icon={pinIcon}
                zIndexOffset={1500}
            >
                <Popup>{`${(this.props.position)[0].toFixed(3)}, ${(this.props.position)[1].toFixed(3)}`}</Popup>
                <Tooltip 
                    permanent={true}
                    direction='right'
                >
                    {this.props.id}
                </Tooltip>
            </Marker>
        ); 
    }

    /**
   * @brief sends the new waypoint location to the ros node
   */
    _onDragend(e){
        let latlng = [e.target.getLatLng().lat, e.target.getLatLng().lng];
        this.props.socket.emit('wayp-moved',this.props.id, latlng);
    } 
}

export default WaypMarker