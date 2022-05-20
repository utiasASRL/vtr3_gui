import clsx from "clsx";
import React from "react";

import Box from "@material-ui/core/Box";
import Button from "@material-ui/core/Button";
import Card from "@material-ui/core/Card";
import ClearIcon from "@material-ui/icons/Clear";
import UnfoldMoreIcon from "@material-ui/icons/UnfoldMore";
import Typography from "@material-ui/core/Typography";
import { withStyles } from "@material-ui/core/styles";
import { sortableHandle } from "react-sortable-hoc";

const DragHandle = sortableHandle(() => <UnfoldMoreIcon fontSize={"large"} />);

class Waypoint extends React.Component{
    render(){
        const {
            waypoint,
            className,
            removeWayp
        } = this.props;

        return(
            <Card className={clsx(className)} style={{ width: "100%" }}>
                <Box width={1} height={1} display={"flex"} flexDirection={"row"}>
                    <Box width={0.1} my={"auto"}>
                        <DragHandle></DragHandle>
                    </Box>
                        
                    

                    <Box display={"flex"} width={1} m={1}>
                        <Box display={"flex"} width={0.5} mr={0.5}>
                            <Typography variant="button">
                                {"Lat: " + waypoint.lat}
                            </Typography>
                        </Box>
                        <Box display={"flex"} width={0.5} ml={0.5}>
                            <Typography variant="button">
                                {"Lon: " + waypoint.lon}
                            </Typography>
                        </Box>
                        <ClearIcon
                            style={{color:'red'}}
                            onClick={(e) => removeWayp()}
                        ></ClearIcon>
                    </Box>

                    
                    
                </Box>
            </Card>
        );
    }
}

export default Waypoint;