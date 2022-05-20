//form to add a waypoint

import React from 'react';

import ArrowBackIcon from "@material-ui/icons/ArrowBack";
import Box from "@material-ui/core/Box";
import Button from "@material-ui/core/Button";
import Card from "@material-ui/core/Card";
import CheckIcon from "@material-ui/icons/Check";
import ClearIcon from "@material-ui/icons/Clear";
import FormControl from "@material-ui/core/FormControl";
import IconButton from "@material-ui/core/IconButton";
import InputAdornment from "@material-ui/core/InputAdornment";
import InputLabel from "@material-ui/core/InputLabel";
import MenuItem from "@material-ui/core/MenuItem";
import Select from "@material-ui/core/Select";
import TextField from "@material-ui/core/TextField";
import { withStyles } from "@material-ui/core/styles";

class AddWaypoint extends React.Component {
    constructor(props){
        super(props);

        this.state = {
            disabled: false, // Disable user inputs while waiting for server response.
            lat: 0,
            lon: 0,
        };
    }

    render(){
        return (
            <Card>
                {/* Submit goal */}
                <Box width={100} m={1}>
                    <Button
                        color={"secondary"}
                        fullWidth={true}
                        disabled={this.state.disabled}
                        disableElevation={true}
                        size="small"
                        startIcon={<CheckIcon />}
                        variant={"contained"}
                        onClick={this._submitWayp.bind(this)}
                    >
                        Add Waypoint
                    </Button>
                </Box>
                {/* Get input lat and lon */}
                <Box
                    mx={1}
                    mb={1}
                    display={"flex"}
                    justifyContent={"center"}
                    flexDirection={"row"}
                >
                    <Box mx={0.5} display={"flex"} justifyContent={"center"}>
                        <TextField
                            disabled={this.state.disabled}
                            fullWidth={true}
                            label="Latitude"
                            onChange={this._setLat.bind(this)}
                            value={this.state.lat}
                        />
                    </Box>
                    <Box mx={0.5} display={"flex"} justifyContent={"center"}>
                        <TextField
                            disabled={this.state.disabled}
                            fullWidth={true}
                            label="Longitude"
                            onChange={this._setLon.bind(this)}
                            value={this.state.lon}
                        />
                    </Box>
                </Box>
        
            </Card>
           
        );
    }

    /**
   * @brief Calls GoalManager to submit the goal and disables further
   * modification until reset.
   */
  _submitWayp() {
    //todo
    
  }

  /** @brief Sets lat */
  _setLat(e) {
    this.setState({ lat: e.target.value });
  }

  /** @brief Sets lat */
  _setLon(e) {
    this.setState({ lon: e.target.value });
  }
}

export default AddWaypoint;