# vtr3_ui

This user interface can but is not meant to run standalone. Include the UI into your projects. For VTR3, follow the installation instructions [there](https://github.com/utiasASRL/vtr3). If you just want to get a sense of how it works, follow the installation instructions below, and choose project "None" at start up.

## Installation

Commands below assumes you are at the **root** directory of this repository.

### Dependencies

```bash
sudo apt install -y nodejs npm protobuf-compiler  # system deps
pip3 install flask flask_socketio eventlet python-socketio python-socketio[client] websocket-client  # python deps
```
```bash
protoc --python_out=`pwd` *.proto  # build protobuf messages (this is optional since we commit the resulting python scripts, run this command after you make any change to the proto files)
cd vtr-ui && npm install .# install dependencies
```

## Modify dependencies.js

```bash
cd vtr-ui/node_modules/babel-preset-react-app/
```
- then open dependencies.js, change it to the following
```javascript
/**
 * Copyright (c) 2015-present, Facebook, Inc.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
'use strict';

const path = require('path');

const validateBoolOption = (name, value, defaultValue) => {
  if (typeof value === 'undefined') {
    value = defaultValue;
  }

  if (typeof value !== 'boolean') {
    throw new Error(`Preset react-app: '${name}' option must be a boolean.`);
  }

  return value;
};

module.exports = function(api, opts) {
  if (!opts) {
    opts = {};
  }

  // This is similar to how `env` works in Babel:
  // https://babeljs.io/docs/usage/babelrc/#env-option
  // We are not using `env` because it’s ignored in versions > babel-core@6.10.4:
  // https://github.com/babel/babel/issues/4539
  // https://github.com/facebook/create-react-app/issues/720
  // It’s also nice that we can enforce `NODE_ENV` being specified.
  var env = process.env.BABEL_ENV || process.env.NODE_ENV;
  var isEnvDevelopment = env === 'development';
  var isEnvProduction = env === 'production';
  var isEnvTest = env === 'test';

  var areHelpersEnabled = validateBoolOption('helpers', opts.helpers, false);
  var useAbsoluteRuntime = validateBoolOption(
    'absoluteRuntime',
    opts.absoluteRuntime,
    true
  );

  var absoluteRuntimePath = undefined;
  if (useAbsoluteRuntime) {
    absoluteRuntimePath = path.dirname(
      require.resolve('@babel/runtime/package.json')
    );
  }

  if (!isEnvDevelopment && !isEnvProduction && !isEnvTest) {
    throw new Error(
      'Using `babel-preset-react-app` requires that you specify `NODE_ENV` or ' +
        '`BABEL_ENV` environment variables. Valid values are "development", ' +
        '"test", and "production". Instead, received: ' +
        JSON.stringify(env) +
        '.'
    );
  }

  return {
    // Babel assumes ES Modules, which isn't safe until CommonJS
    // dies. This changes the behavior to assume CommonJS unless
    // an `import` or `export` is present in the file.
    // https://github.com/webpack/webpack/issues/4039#issuecomment-419284940
    sourceType: 'unambiguous',
    presets: [
      isEnvTest && [
        // ES features necessary for user's Node version
        require('@babel/preset-env').default,
        {
          targets: {
            node: 'current',
          },
          // Do not transform modules to CJS
          modules: false,
          // Exclude transforms that make all code slower
          exclude: ['transform-typeof-symbol'],
        },
      ],
      (isEnvProduction || isEnvDevelopment) && [
        // Latest stable ECMAScript features
        require('@babel/preset-env').default,
        {
          // Allow importing core-js in entrypoint and use browserlist to select polyfills
          useBuiltIns: 'entry',
          // Set the corejs version we are using to avoid warnings in console
          // This will need to change once we upgrade to corejs@3
          corejs: 3,
          // Do not transform modules to CJS
          modules: false,
          // Exclude transforms that make all code slower
          exclude: ['transform-typeof-symbol'],
        },
      ],
      //⚠️ added preset-flow
       [require('@babel/preset-flow').default],
    ].filter(Boolean),
    plugins: [
      // Disabled as it's handled automatically by preset-env, and `selectiveLoose` isn't
      // yet merged into babel: https://github.com/babel/babel/pull/9486
      // Related: https://github.com/facebook/create-react-app/pull/8215
      // [
      //   require('@babel/plugin-transform-destructuring').default,
      //   {
      //     // Use loose mode for performance:
      //     // https://github.com/facebook/create-react-app/issues/5602
      //     loose: false,
      //     selectiveLoose: [
      //       'useState',
      //       'useEffect',
      //       'useContext',
      //       'useReducer',
      //       'useCallback',
      //       'useMemo',
      //       'useRef',
      //       'useImperativeHandle',
      //       'useLayoutEffect',
      //       'useDebugValue',
      //     ],
      //   },
      // ],
      // Polyfills the runtime needed for async/await, generators, and friends
      // https://babeljs.io/docs/en/babel-plugin-transform-runtime
      [
        require('@babel/plugin-transform-runtime').default,
        {
          corejs: false,
          helpers: areHelpersEnabled,
          // By default, babel assumes babel/runtime version 7.0.0-beta.0,
          // explicitly resolving to match the provided helper functions.
          // https://github.com/babel/babel/issues/10261
          version: require('@babel/runtime/package.json').version,
          regenerator: true,
          // https://babeljs.io/docs/en/babel-plugin-transform-runtime#useesmodules
          // We should turn this on once the lowest version of Node LTS
          // supports ES Modules.
          useESModules: isEnvDevelopment || isEnvProduction,
          // Undocumented option that lets us encapsulate our runtime, ensuring
          // the correct version is used
          // https://github.com/babel/babel/blob/090c364a90fe73d36a30707fc612ce037bdbbb24/packages/babel-plugin-transform-runtime/src/index.js#L35-L42
          absoluteRuntime: absoluteRuntimePath,
        },
      ],
      [
        require('@babel/plugin-transform-flow-strip-types').default,
        false,
      ],
       [
        require('@babel/plugin-proposal-class-properties').default,
        {
          loose: true,
        },
      ],
    ].filter(Boolean),
  };
};

```
```bash
npm run build # "compile" javascripts
```

## Launch

In one terminal,

```bash
python3 web_server.py  # web server that hosts the web page and handle some requests from the frontend
```

In another terminal,

```bash
python3 socket_server.py  # socket io server that passes data between the frontend and other systems, mainly used to send updates from vtr to the frontend
```

Open your brower and navigate to `http://localhost:5200`, and select "None" when choosing project.

## To Developers

VTR3 user interface is a single page web app tested with Google Chrome and Firefox. The backend server uses python [Flask](https://flask.palletsprojects.com/en/1.1.x/), and the implementation is [here](./web_server.py). The frontend is written in html, javascript and css, and it mainly uses the following libraries:

- [React](https://reactjs.org/): A JavaScript library for building user interfaces.
  - Used for designing the entire user interface.
  - We choose this library because it is the most popular one at the moment.
- [Leaflet](https://leafletjs.com/) and [React-Leaflet](https://react-leaflet.js.org/): An open-source JavaScript library for mobile-friendly interactive maps and its React wrapper.
  - Used for displaying the background map and pose graph.
- [Socket.IO](https://socket.io/): Real-time communication.
  - Used for getting real-time updates from the main VT&R process.
- [Protocol Buffers](https://developers.google.com/protocol-buffers): Language-neutral, platform-neutral extensible mechanism for serializing structured data.
  - Used for serializing graph and robot updates.
- [Webpack](https://webpack.js.org/) and [Babel](https://babeljs.io/): JavaScript bundler and compiler, respectively.
  - Used for transforming and optimizing the frontend code so that we can serve it using our backend.
  - We choose these two packages because they are the default of [Create React App](https://create-react-app.dev/), which is used to start our UI development.

Always make sure the packages being used are the latest, upgrade packages to the latest version periodically (`npx npm-check-updates -u`). See README inside `vtr-ui` for more information about how to build the frontend. Currently we do not support using the default ES6 server provided by [Create React App](https://create-react-app.dev/), so you have to debug UI with VTR running.
