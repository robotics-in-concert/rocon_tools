
function queryFieldByName(name) {
    name = name.replace(/[\[]/, "\\[").replace(/[\]]/, "\\]");
    var regex = new RegExp("[\\?&]" + name + "=([^&#]*)"),
    results = regex.exec(window.location.search);
    // Decode it, note the special attention to '+' symbols to spaces.
    return results == null ? "" : decodeURIComponent(results[1].replace(/\+/g, " "));
}

function queryInteractionParameters() {
    // TODO : remove rosbridge_address adn rosbrdige_port variables
    // from here as they are typically not web app relevant
    param_string = queryFieldByName('params')
    // could use some error checking to make sure it doesn't return null here.
    return JSON.parse(param_string)
}

function queryInteractionsRosbridgeURI() {
    // TODO : remove rosbridge_address adn rosbrdige_port variables
    // from here as they are typically not web app relevant
	param_string = queryFieldByName('params')
	// could use some error checking to make sure it doesn't return null here.
	params = JSON.parse(param_string)
	// check if rosbridge_address and rosbridge_port are not present and
	// substitute with localhost:9090 if so.
	return 'ws://' + params['rosbridge_address'] + ':' + params['rosbridge_port'] 
}

function queryInteractionRemaps() {
	// This goes quite a bit further than directly returning the remaps json string
	// as an object. It takes th interactions specification and parses the list
	// converting it into a dictionary with keys as the remap froms and values
	// as the remap tos'. This is useful for working with, wherase the former
	// is useful in terms of ros api.

	remaps_string = queryFieldByName('remaps')
	// could use some error checking to make sure it doesn't return null here.
	o = JSON.parse(remaps_string)
	// convert this from the list of 'remap_from': 'xyz', 'remap_to': 'zyx' dictionaries to
	// a single dictionary with remap_from keys and remap_to pairs.
	remaps = {}
	for (var i = 0; i < o.length; i++) {
        remaps[o[i]['remap_from']] = o[i]['remap_to']
    }
    return remaps
}
