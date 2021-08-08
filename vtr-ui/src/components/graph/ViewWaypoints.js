//component to view the current waypoints & add waypoint
import clsx from "clsx";
import shortid from "shortid";
import React from "react";

import AddIcon from "@material-ui/icons/Add";
import ArrowBackIosIcon from "@material-ui/icons/ArrowBackIos";
import ArrowForwardIosIcon from "@material-ui/icons/ArrowForwardIos";
import Box from "@material-ui/core/Box";
import Button from "@material-ui/core/Button";
import ClearIcon from "@material-ui/icons/Clear";
import Drawer from "@material-ui/core/Drawer";
import PauseIcon from "@material-ui/icons/Pause";
import PlayArrowIcon from "@material-ui/icons/PlayArrow";
import StopIcon from "@material-ui/icons/Stop";
import StorageIcon from "@material-ui/icons/Storage";
import { withStyles } from "@material-ui/core/styles";
import {
  sortableContainer,
  sortableElement,
  arrayMove,
} from "react-sortable-hoc";

import AddWaypoint from './AddWaypoint'
import Waypoint from './Waypoint'

const Wayp = sortableElement((props) => {
    return (
      <Box width={1}>
        <Waypoint
          waypoint={props.waypoint}
          id={props.id}
          removeWayp={props.removeWayp}
        ></Waypoint>
      </Box>
    );
});

const WaypContainer = sortableContainer((props) => {
    const { className, maxHeight, top } = props;
    return (
      <Box
        className={className}
        width={1}
        maxHeight={maxHeight}
        style={{ overflowY: "scroll", marginTop: top }}
      >
        {props.children}
      </Box>
    );
});

// Style
const currWaypointCardHeight = 160;
const waypFormHeight = 300;
const waypPanelWidth = 300;
const topButtonHeight = 10;
const transitionDuration = 300;
const styles = (theme) => ({
  waypPanelButton: {
    transition: theme.transitions.create(["left"], {
      duration: transitionDuration,
    }),
  },
  waypContainer: {},
  waypContainerHelper: {
    zIndex: 2000, // \todo This is a magic number.
  },
});

class ViewWaypoints extends React.Component{
    constructor(props){
        super(props);

        this.state = {
            waypPanelOpen: false,
            windowHeight : 0,
            addingWayp: false,
            waypoints : [
                {
                    lat: 10,
                    lon: 10,
                }
                ,
                {
                    lat: 20,
                    lon: 20,
                }
                ,
                {
                    lat: 40,
                    lon: 40,
                },
            ]
        };

    }

    componentDidMount(){
        // Auto-adjusts window height
        window.addEventListener("resize", this._updateWindowHeight.bind(this));
        this._updateWindowHeight();
    }
    
    componentWillUnmount(){
        window.removeEventListener("resize", this._updateWindowHeight.bind(this));
    }

    render(){
        const {
            classes,
            className,
        } = this.props;

        const{
            waypoints,
            waypPanelOpen,
            addingWayp,
            windowHeight,
        } = this.state;

        return (
            <>
                {/* Button to open/close the waypoint drawer */}
                <Box
                    className={classes.waypPanelButton}
                    position={"absolute"}
                    top={0}
                    left={waypPanelOpen ? waypPanelWidth + 10 : 0}
                    zIndex={1000}
                    m={0.5}
                    width={150}
                >
                    <Button
                        color={waypPanelOpen ? "secondary" : "primary"}
                        disableElevation={true}
                        variant={"contained"}
                        fullWidth={true}
                        startIcon={<StorageIcon />}
                        endIcon={
                            waypPanelOpen ? <ArrowBackIosIcon /> : <ArrowForwardIosIcon />
                        }
                        onClick={this._toggleWaypPanel.bind(this)}
                    >
                        Waypoints
                    </Button>
                </Box>

                {/* The drawer that shows current list of waypoints in queue and waypoint addition form */}
                <Drawer
                    className={clsx(className)}
                    variant="persistent"
                    anchor="left"
                    open={waypPanelOpen}
                    transitionDuration={transitionDuration}
                    PaperProps={{
                        elevation: 0,
                        style: {
                            backgroundColor: "rgba(255, 255, 255, 0.0)",
                        },
                    }}
                    >
                    
                    {/* List of waypoints in queue */}
                    <Box
                        width={waypPanelWidth}
                        display={"flex"}
                        justifyContent={"center"}
                        flexDirection={"row"}
                        m={0.5}
                    >
                        <WaypContainer
                            className={classes.waypContainer}
                            helperClass={classes.waypContainerHelper}
                            lockAxis = "y"
                            distance={2}
                            onSortEnd={(e) => {
                                this._moveWayp(e.oldIndex, e.newIndex);
                            }}
                            useDragHandle
                            // Cannot pass through className because it depends on state.
                            // \todo jsx error that causes incorrect return from ?: operator?
                            maxHeight={
                                windowHeight - waypFormHeight
                            }
                            // Cannot pass through className because it depends on state.
                            top={0}                            
                        >
                            {waypoints.map((waypoint, index) => {
                                return (
                                    <Wayp
                                        key={shortid.generate()}
                                        waypoint={waypoint}
                                        id={index}
                                        index={index}
                                        removeWayp={this._removeWayp.bind(this)}
                                    />
                                );
                            })}
                        </WaypContainer>
                    </Box>

                    {/* Waypoint addition form */}
                    <Box width={waypPanelWidth} m={0.5}>
                        {addingWayp && (
                            <AddWaypoint
                                submit={this._submitWayp.bind(this)}
                            ></AddWaypoint>
                        )}
                    </Box>
                    {/* Waypoint addition button */}
                    <Box
                        width={waypPanelWidth}
                        display={"flex"}
                        justifyContent={"center"}
                        m={0.5}
                    >
                        <Button
                            disableElevation={true}
                            color={addingWayp ? "secondary" : "primary"}
                            size={"small"}
                            startIcon={addingWayp ? <ClearIcon /> : <AddIcon />}
                            variant={"contained"}
                            onClick={this._toggleWaypForm.bind(this)}
                        >
                            {addingWayp ? "Cancel" : "Add Waypoint"}
                        </Button>
                    </Box>
                </Drawer>
            </>
        );
    }

    /** Shows/hides the goal addition form. */
    _toggleWaypForm() {
        this.setState((state) => ({
            addingWayp: !state.addingWayp,
        }));
    }

    /** Shows/hides the waypoint panel. */
    _toggleWaypPanel(){
        this.setState((state) => ({ waypPanelOpen: !state.waypPanelOpen }));
    }

    /**
   * @brief Submits the user specified waypoint to add via SocketIO.
   */
    _submitWayp() {
        //todo
    }

  /**
   * @brief Deletes the user specified waypoint via SocketIO.
   */
   _removeWayp() {
    //todo
  }

  _moveWayp(oldIdx, newIdx){
    //todo
  }

  _updateWindowHeight(){
    this.setState({ windowHeight: window.innerHeight });
  }

}

export default withStyles(styles)(ViewWaypoints);
