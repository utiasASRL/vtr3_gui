import React from "react";
import {Marker, Tooltip, Popup} from 'react-leaflet';
import L, { icon, LatLng } from "leaflet";
import pinSvg from '../../images/pin-icon.svg'
import circleSvg from '../../images/Red_circle.svg'

const pinIcon = new L.Icon({
    iconUrl: pinSvg,
    iconSize: new L.Point(49, 50),
    iconAnchor: [24, 49]
  });

  const circleIcon = new L.Icon({
    iconUrl: circleSvg,
    iconSize: new L.Point(49, 50),
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
                icon={circleIcon}
                zIndexOffset={1500}
                oncontextmenu={this.props.oncontext}
            >
                <Popup offset={[0, -24]}>{`${(this.props.position)[0].toFixed(3)}, ${(this.props.position)[1].toFixed(3)}`}</Popup>
                <Tooltip 
                    permanent={true}
                    direction='right'
                    offset={[0, -24]}
                >
                    {this.props.id}
                </Tooltip>
            </Marker>
                
        ); 
    }
    
    _onDragend(e){
        let latlng = [e.target.getLatLng().lat, e.target.getLatLng().lng];
        this.props.socket.emit('wayp-moved',this.props.id, latlng);
    } 
}

export default WaypMarker