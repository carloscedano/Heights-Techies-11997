/**
 * @license
 * Copyright 2017 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @fileoverview FTC robot blocks related to Vuforia
 * @author lizlooney@google.com (Liz Looney)
 */

// The following are generated dynamically in HardwareUtil.fetchJavaScriptForHardware():
// miscIdentifierForJavaScript
// The following are defined in vars.js:
// createNonEditableField
// functionColor
// getPropertyColor

function vuforia_initialize_withCameraDirection_JavaScript(block, vuforiaLicenseKey, identifier) {
  var cameraDirection = Blockly.JavaScript.valueToCode(
      block, 'CAMERA_DIRECTION', Blockly.JavaScript.ORDER_COMMA);
  var useExtendedTracking = Blockly.JavaScript.valueToCode(
      block, 'USE_EXTENDED_TRACKING', Blockly.JavaScript.ORDER_COMMA);
  var enableCameraMonitoring = Blockly.JavaScript.valueToCode(
      block, 'ENABLE_CAMERA_MONITORING', Blockly.JavaScript.ORDER_COMMA);
  var cameraMonitorFeedback = Blockly.JavaScript.valueToCode(
      block, 'CAMERA_MONITOR_FEEDBACK', Blockly.JavaScript.ORDER_COMMA);
  var dx = Blockly.JavaScript.valueToCode(
      block, 'PHONE_LOCATION_ON_ROBOT_DX', Blockly.JavaScript.ORDER_COMMA);
  var dy = Blockly.JavaScript.valueToCode(
      block, 'PHONE_LOCATION_ON_ROBOT_DY', Blockly.JavaScript.ORDER_COMMA);
  var dz = Blockly.JavaScript.valueToCode(
      block, 'PHONE_LOCATION_ON_ROBOT_DZ', Blockly.JavaScript.ORDER_COMMA);
  var xAngle = Blockly.JavaScript.valueToCode(
      block, 'PHONE_LOCATION_ON_ROBOT_X_ANGLE', Blockly.JavaScript.ORDER_COMMA);
  var yAngle = Blockly.JavaScript.valueToCode(
      block, 'PHONE_LOCATION_ON_ROBOT_Y_ANGLE', Blockly.JavaScript.ORDER_COMMA);
  var zAngle = Blockly.JavaScript.valueToCode(
      block, 'PHONE_LOCATION_ON_ROBOT_Z_ANGLE', Blockly.JavaScript.ORDER_COMMA);
  var useCompetitionFieldTargetLocations = Blockly.JavaScript.valueToCode(
      block, 'USE_COMPETITION_FIELD_TARGET_LOCATIONS', Blockly.JavaScript.ORDER_COMMA);
  // During the pull-request review, this block didn't always have a
  // USE_COMPETITION_FIELD_TARGET_LOCATIONS input. If useCompetitionFieldTargetLocations is '',
  // explicitly set it to 'true'.
  if (useCompetitionFieldTargetLocations == '') {
    useCompetitionFieldTargetLocations = 'true';
  }
  return identifier + '.initialize_withCameraDirection(' + vuforiaLicenseKey + ', ' +
      cameraDirection + ', ' + useExtendedTracking + ', ' +
      enableCameraMonitoring + ', ' + cameraMonitorFeedback + ', ' +
      dx + ', ' + dy + ', ' + dz + ', ' + xAngle + ', ' + yAngle + ', ' + zAngle + ', ' +
      useCompetitionFieldTargetLocations + ');\n';
}

function vuforia_initialize_withCameraDirection_FtcJava(block, vuforiaLicenseKey, className) {
  var identifier = importDeclareAssignObsolete(block, null, className);
  var cameraDirection = Blockly.FtcJava.valueToCode(
      block, 'CAMERA_DIRECTION', Blockly.FtcJava.ORDER_COMMA);
  var useExtendedTracking = Blockly.FtcJava.valueToCode(
      block, 'USE_EXTENDED_TRACKING', Blockly.FtcJava.ORDER_COMMA);
  var enableCameraMonitoring = Blockly.FtcJava.valueToCode(
      block, 'ENABLE_CAMERA_MONITORING', Blockly.FtcJava.ORDER_COMMA);
  var cameraMonitorFeedback = Blockly.FtcJava.valueToCode(
      block, 'CAMERA_MONITOR_FEEDBACK', Blockly.FtcJava.ORDER_COMMA);
  var dx = Blockly.FtcJava.valueToCode(
      block, 'PHONE_LOCATION_ON_ROBOT_DX', Blockly.FtcJava.ORDER_COMMA);
  var dy = Blockly.FtcJava.valueToCode(
      block, 'PHONE_LOCATION_ON_ROBOT_DY', Blockly.FtcJava.ORDER_COMMA);
  var dz = Blockly.FtcJava.valueToCode(
      block, 'PHONE_LOCATION_ON_ROBOT_DZ', Blockly.FtcJava.ORDER_COMMA);
  var xAngle = Blockly.FtcJava.valueToCode(
      block, 'PHONE_LOCATION_ON_ROBOT_X_ANGLE', Blockly.FtcJava.ORDER_COMMA);
  var yAngle = Blockly.FtcJava.valueToCode(
      block, 'PHONE_LOCATION_ON_ROBOT_Y_ANGLE', Blockly.FtcJava.ORDER_COMMA);
  var zAngle = Blockly.FtcJava.valueToCode(
      block, 'PHONE_LOCATION_ON_ROBOT_Z_ANGLE', Blockly.FtcJava.ORDER_COMMA);
  var useCompetitionFieldTargetLocations = Blockly.FtcJava.valueToCode(
      block, 'USE_COMPETITION_FIELD_TARGET_LOCATIONS', Blockly.FtcJava.ORDER_COMMA);
  // During the pull-request review, this block didn't always have a
  // USE_COMPETITION_FIELD_TARGET_LOCATIONS input. If useCompetitionFieldTargetLocations is '',
  // explicitly set it to 'true'.
  if (useCompetitionFieldTargetLocations == '') {
    useCompetitionFieldTargetLocations = 'true';
  }
  return identifier + '.initialize(\n' +
      Blockly.FtcJava.INDENT_CONTINUE + vuforiaLicenseKey + ', // vuforiaLicenseKey\n' +
      Blockly.FtcJava.INDENT_CONTINUE + cameraDirection + ', // cameraDirection\n' +
      Blockly.FtcJava.INDENT_CONTINUE + useExtendedTracking + ', // useExtendedTracking\n' +
      Blockly.FtcJava.INDENT_CONTINUE + enableCameraMonitoring + ', // enableCameraMonitoring\n' +
      Blockly.FtcJava.INDENT_CONTINUE + cameraMonitorFeedback + ', // cameraMonitorFeedback\n' +
      Blockly.FtcJava.INDENT_CONTINUE + dx + ', // dx\n' +
      Blockly.FtcJava.INDENT_CONTINUE + dy + ', // dy\n' +
      Blockly.FtcJava.INDENT_CONTINUE + dz + ', // dz\n' +
      Blockly.FtcJava.INDENT_CONTINUE + xAngle + ', // xAngle\n' +
      Blockly.FtcJava.INDENT_CONTINUE + yAngle + ', // yAngle\n' +
      Blockly.FtcJava.INDENT_CONTINUE + zAngle + ', // zAngle\n' +
      Blockly.FtcJava.INDENT_CONTINUE + useCompetitionFieldTargetLocations + '); // useCompetitionFieldTargetLocations\n';
}

function vuforia_initialize_withWebcam_JavaScript(block, vuforiaLicenseKey, identifier) {
  var cameraName = Blockly.JavaScript.valueToCode(
      block, 'CAMERA_NAME', Blockly.JavaScript.ORDER_COMMA);
  var webcamCalibrationFilename = Blockly.JavaScript.valueToCode(
      block, 'WEBCAM_CALIBRATION_FILE', Blockly.JavaScript.ORDER_COMMA);
  var useExtendedTracking = Blockly.JavaScript.valueToCode(
      block, 'USE_EXTENDED_TRACKING', Blockly.JavaScript.ORDER_COMMA);
  var enableCameraMonitoring = Blockly.JavaScript.valueToCode(
      block, 'ENABLE_CAMERA_MONITORING', Blockly.JavaScript.ORDER_COMMA);
  var cameraMonitorFeedback = Blockly.JavaScript.valueToCode(
      block, 'CAMERA_MONITOR_FEEDBACK', Blockly.JavaScript.ORDER_COMMA);
  var dx = Blockly.JavaScript.valueToCode(
      block, 'CAMERA_LOCATION_ON_ROBOT_DX', Blockly.JavaScript.ORDER_COMMA);
  var dy = Blockly.JavaScript.valueToCode(
      block, 'CAMERA_LOCATION_ON_ROBOT_DY', Blockly.JavaScript.ORDER_COMMA);
  var dz = Blockly.JavaScript.valueToCode(
      block, 'CAMERA_LOCATION_ON_ROBOT_DZ', Blockly.JavaScript.ORDER_COMMA);
  var xAngle = Blockly.JavaScript.valueToCode(
      block, 'CAMERA_LOCATION_ON_ROBOT_X_ANGLE', Blockly.JavaScript.ORDER_COMMA);
  var yAngle = Blockly.JavaScript.valueToCode(
      block, 'CAMERA_LOCATION_ON_ROBOT_Y_ANGLE', Blockly.JavaScript.ORDER_COMMA);
  var zAngle = Blockly.JavaScript.valueToCode(
      block, 'CAMERA_LOCATION_ON_ROBOT_Z_ANGLE', Blockly.JavaScript.ORDER_COMMA);
  var useCompetitionFieldTargetLocations = Blockly.JavaScript.valueToCode(
      block, 'USE_COMPETITION_FIELD_TARGET_LOCATIONS', Blockly.JavaScript.ORDER_COMMA);
  return identifier + '.initialize_withWebcam(' +
      cameraName + ', ' + webcamCalibrationFilename + ', ' + useExtendedTracking + ', ' +
      enableCameraMonitoring + ', ' + cameraMonitorFeedback + ', ' +
      dx + ', ' + dy + ', ' + dz + ', ' + xAngle + ', ' + yAngle + ', ' + zAngle + ', ' +
      useCompetitionFieldTargetLocations + ');\n';
}

function vuforia_initialize_withWebcam_FtcJava(block, className) {
  var identifier = importDeclareAssignObsolete(block, null, className);
  var cameraName = Blockly.FtcJava.valueToCode(
      block, 'CAMERA_NAME', Blockly.FtcJava.ORDER_COMMA);
  var webcamCalibrationFilename = Blockly.FtcJava.valueToCode(
      block, 'WEBCAM_CALIBRATION_FILE', Blockly.FtcJava.ORDER_COMMA);
  var useExtendedTracking = Blockly.FtcJava.valueToCode(
      block, 'USE_EXTENDED_TRACKING', Blockly.FtcJava.ORDER_COMMA);
  var enableCameraMonitoring = Blockly.FtcJava.valueToCode(
      block, 'ENABLE_CAMERA_MONITORING', Blockly.FtcJava.ORDER_COMMA);
  var cameraMonitorFeedback = Blockly.FtcJava.valueToCode(
      block, 'CAMERA_MONITOR_FEEDBACK', Blockly.FtcJava.ORDER_COMMA);
  var dx = Blockly.FtcJava.valueToCode(
      block, 'CAMERA_LOCATION_ON_ROBOT_DX', Blockly.FtcJava.ORDER_COMMA);
  var dy = Blockly.FtcJava.valueToCode(
      block, 'CAMERA_LOCATION_ON_ROBOT_DY', Blockly.FtcJava.ORDER_COMMA);
  var dz = Blockly.FtcJava.valueToCode(
      block, 'CAMERA_LOCATION_ON_ROBOT_DZ', Blockly.FtcJava.ORDER_COMMA);
  var xAngle = Blockly.FtcJava.valueToCode(
      block, 'CAMERA_LOCATION_ON_ROBOT_X_ANGLE', Blockly.FtcJava.ORDER_COMMA);
  var yAngle = Blockly.FtcJava.valueToCode(
      block, 'CAMERA_LOCATION_ON_ROBOT_Y_ANGLE', Blockly.FtcJava.ORDER_COMMA);
  var zAngle = Blockly.FtcJava.valueToCode(
      block, 'CAMERA_LOCATION_ON_ROBOT_Z_ANGLE', Blockly.FtcJava.ORDER_COMMA);
  var useCompetitionFieldTargetLocations = Blockly.FtcJava.valueToCode(
      block, 'USE_COMPETITION_FIELD_TARGET_LOCATIONS', Blockly.FtcJava.ORDER_COMMA);
  return identifier + '.initialize(\n' +
      Blockly.FtcJava.INDENT_CONTINUE + '"", // vuforiaLicenseKey\n' +
      Blockly.FtcJava.INDENT_CONTINUE + cameraName + ', // cameraName\n' +
      Blockly.FtcJava.INDENT_CONTINUE + webcamCalibrationFilename + ', // webcamCalibrationFilename\n' +
      Blockly.FtcJava.INDENT_CONTINUE + useExtendedTracking + ', // useExtendedTracking\n' +
      Blockly.FtcJava.INDENT_CONTINUE + enableCameraMonitoring + ', // enableCameraMonitoring\n' +
      Blockly.FtcJava.INDENT_CONTINUE + cameraMonitorFeedback + ', // cameraMonitorFeedback\n' +
      Blockly.FtcJava.INDENT_CONTINUE + dx + ', // dx\n' +
      Blockly.FtcJava.INDENT_CONTINUE + dy + ', // dy\n' +
      Blockly.FtcJava.INDENT_CONTINUE + dz + ', // dz\n' +
      Blockly.FtcJava.INDENT_CONTINUE + xAngle + ', // xAngle\n' +
      Blockly.FtcJava.INDENT_CONTINUE + yAngle + ', // yAngle\n' +
      Blockly.FtcJava.INDENT_CONTINUE + zAngle + ', // zAngle\n' +
      Blockly.FtcJava.INDENT_CONTINUE + useCompetitionFieldTargetLocations + '); // useCompetitionFieldTargetLocations\n';
}

function vuforia_activate_JavaScript(block, identifier) {
  return identifier + '.activate();\n';
}

function vuforia_activate_FtcJava(block, className) {
  var identifier = importDeclareAssignObsolete(block, null, className);
  return identifier + '.activate();\n';
}

function vuforia_deactivate_JavaScript(block, identifier) {
  return identifier + '.deactivate();\n';
}

function vuforia_deactivate_FtcJava(block, className) {
  var identifier = importDeclareAssignObsolete(block, null, className);
  return identifier + '.deactivate();\n';
}

function vuforia_setActiveCamera_JavaScript(block, identifier) {
  var cameraName = Blockly.JavaScript.valueToCode(
      block, 'CAMERA_NAME', Blockly.JavaScript.ORDER_COMMA);
  return identifier + '.setActiveCamera(' + cameraName + ');\n';
}

function vuforia_setActiveCamera_FtcJava(block, className) {
  var identifier = importDeclareAssignObsolete(block, null, className);
  var cameraName = Blockly.FtcJava.valueToCode(
      block, 'CAMERA_NAME', Blockly.FtcJava.ORDER_COMMA);
  return identifier + '.setActiveCamera(' + cameraName + ');\n';
}

function vuforia_track_JavaScript(block, identifier) {
  var trackableName = Blockly.JavaScript.valueToCode(
      block, 'TRACKABLE_NAME', Blockly.JavaScript.ORDER_NONE);
  var code = 'JSON.parse(' + identifier + '.track(' + trackableName + '))';
  return [code, Blockly.JavaScript.ORDER_FUNCTION_CALL];
}

function vuforia_track_FtcJava(block, className) {
  var identifier = importDeclareAssignObsolete(block, null, className);
  var trackableName = Blockly.FtcJava.valueToCode(
      block, 'TRACKABLE_NAME', Blockly.FtcJava.ORDER_NONE);
  var code = identifier + '.track(' + trackableName + ')';
  return [code, Blockly.FtcJava.ORDER_FUNCTION_CALL];
}

function vuforia_trackPose_JavaScript(block, identifier) {
  var trackableName = Blockly.JavaScript.valueToCode(
      block, 'TRACKABLE_NAME', Blockly.JavaScript.ORDER_NONE);
  var code = 'JSON.parse(' + identifier + '.trackPose(' + trackableName + '))';
  return [code, Blockly.JavaScript.ORDER_FUNCTION_CALL];
}

function vuforia_trackPose_FtcJava(block, className) {
  var identifier = importDeclareAssignObsolete(block, null, className);
  var trackableName = Blockly.FtcJava.valueToCode(
      block, 'TRACKABLE_NAME', Blockly.FtcJava.ORDER_NONE);
  var code = identifier + '.trackPose(' + trackableName + ')';
  return [code, Blockly.FtcJava.ORDER_FUNCTION_CALL];
}

function vuforia_typedEnum_trackableName_JavaScript(block) {
  var code = '"' + block.getFieldValue('TRACKABLE_NAME') + '"';
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
}

function vuforia_typedEnum_trackableName_FtcJava(block) {
  // Even in Java, a trackable name is actually just a string, not an enum.
  var code = '"' + block.getFieldValue('TRACKABLE_NAME') + '"';
  return [code, Blockly.FtcJava.ORDER_ATOMIC];
}

// TrackingResults

Blockly.Blocks['vuforiaTrackingResults_getProperty_String'] = {
  init: function() {
    var PROPERTY_CHOICES = [
        ['Name', 'Name'],
    ];
    this.setOutput(true, 'String');
    this.appendDummyInput()
        .appendField(createNonEditableField('VuforiaTrackingResults'))
        .appendField('.')
        .appendField(new Blockly.FieldDropdown(PROPERTY_CHOICES), 'PROP');
    this.appendValueInput('VUFORIA_TRACKING_RESULTS').setCheck('VuforiaBase.TrackingResults')
        .appendField('vuforiaTrackingResults')
        .setAlign(Blockly.ALIGN_RIGHT);
    this.setColour(getPropertyColor);
    // Assign 'this' to a variable for use in the tooltip closure below.
    var thisBlock = this;
    var TOOLTIPS = [
        ['Name', 'Returns the name of the associated trackable.'],
    ];
    this.setTooltip(function() {
      var key = thisBlock.getFieldValue('PROP');
      for (var i = 0; i < TOOLTIPS.length; i++) {
        if (TOOLTIPS[i][0] == key) {
          return TOOLTIPS[i][1];
        }
      }
      return '';
    });
  }
};

Blockly.JavaScript['vuforiaTrackingResults_getProperty_String'] = function(block) {
  var property = block.getFieldValue('PROP');
  var vuforiaTrackingResults = Blockly.JavaScript.valueToCode(
      block, 'VUFORIA_TRACKING_RESULTS', Blockly.JavaScript.ORDER_MEMBER);
  var code = vuforiaTrackingResults + '.' + property;
  var blockLabel = 'VuforiaTrackingResults.' + block.getField('PROP').getText();
  return wrapJavaScriptCode(code, blockLabel);
};

Blockly.FtcJava['vuforiaTrackingResults_getProperty_String'] = function(block) {
  var property = block.getFieldValue('PROP');
  var vuforiaTrackingResults = Blockly.FtcJava.valueToCode(
      block, 'VUFORIA_TRACKING_RESULTS', Blockly.FtcJava.ORDER_MEMBER);
  var code = vuforiaTrackingResults + '.' + Blockly.FtcJava.makeFirstLetterLowerCase_(property);
  return [code, Blockly.FtcJava.ORDER_MEMBER];
};

Blockly.Blocks['vuforiaTrackingResults_getProperty_Boolean'] = {
  init: function() {
    var PROPERTY_CHOICES = [
        ['IsVisible', 'IsVisible'],
        ['IsUpdatedRobotLocation', 'IsUpdatedRobotLocation'],
    ];
    this.setOutput(true, 'Boolean');
    this.appendDummyInput()
        .appendField(createNonEditableField('VuforiaTrackingResults'))
        .appendField('.')
        .appendField(new Blockly.FieldDropdown(PROPERTY_CHOICES), 'PROP');
    this.appendValueInput('VUFORIA_TRACKING_RESULTS').setCheck('VuforiaBase.TrackingResults')
        .appendField('vuforiaTrackingResults')
        .setAlign(Blockly.ALIGN_RIGHT);
    this.setColour(getPropertyColor);
    // Assign 'this' to a variable for use in the tooltip closure below.
    var thisBlock = this;
    var TOOLTIPS = [
        ['IsVisible', 'Returns true if the associated trackable is visible, false otherwise.'],
        ['IsUpdatedRobotLocation', 'Returns true if these results contain an updated robot location, false otherwise.'],
    ];
    this.setTooltip(function() {
      var key = thisBlock.getFieldValue('PROP');
      for (var i = 0; i < TOOLTIPS.length; i++) {
        if (TOOLTIPS[i][0] == key) {
          return TOOLTIPS[i][1];
        }
      }
      return '';
    });
  }
};

Blockly.JavaScript['vuforiaTrackingResults_getProperty_Boolean'] = function(block) {
  var property = block.getFieldValue('PROP');
  var vuforiaTrackingResults = Blockly.JavaScript.valueToCode(
      block, 'VUFORIA_TRACKING_RESULTS', Blockly.JavaScript.ORDER_MEMBER);
  var code = vuforiaTrackingResults + '.' + property;
  var blockLabel = 'VuforiaTrackingResults.' + block.getField('PROP').getText();
  return wrapJavaScriptCode(code, blockLabel);
};

Blockly.FtcJava['vuforiaTrackingResults_getProperty_Boolean'] = function(block) {
  var property = block.getFieldValue('PROP');
  var vuforiaTrackingResults = Blockly.FtcJava.valueToCode(
      block, 'VUFORIA_TRACKING_RESULTS', Blockly.FtcJava.ORDER_MEMBER);
  var code = vuforiaTrackingResults + '.' + Blockly.FtcJava.makeFirstLetterLowerCase_(property);
  return [code, Blockly.FtcJava.ORDER_MEMBER];
};


Blockly.Blocks['vuforiaTrackingResults_getProperty_Number'] = {
  init: function() {
    var PROPERTY_CHOICES = [
        ['X', 'X'],
        ['Y', 'Y'],
        ['Z', 'Z'],
        ['XAngle', 'XAngle'],
        ['YAngle', 'YAngle'],
        ['ZAngle', 'ZAngle'],
    ];
    this.setOutput(true, 'Number');
    this.appendDummyInput()
        .appendField(createNonEditableField('VuforiaTrackingResults'))
        .appendField('.')
        .appendField(new Blockly.FieldDropdown(PROPERTY_CHOICES), 'PROP');
    this.appendValueInput('VUFORIA_TRACKING_RESULTS').setCheck('VuforiaBase.TrackingResults')
        .appendField('vuforiaTrackingResults')
        .setAlign(Blockly.ALIGN_RIGHT);
    this.setColour(getPropertyColor);
    // Assign 'this' to a variable for use in the closures below.
    var thisBlock = this;
    var TOOLTIPS = [
        ['X', 'Returns the X value of the given VuforiaTrackingResults object.'],
        ['Y', 'Returns the Y value of the given VuforiaTrackingResults object.'],
        ['Z', 'Returns the Z value of the given VuforiaTrackingResults object.'],
        ['XAngle', 'Returns the X rotation angle of the given VuforiaTrackingResults object.'],
        ['YAngle', 'Returns the Y rotation angle of the given VuforiaTrackingResults object.'],
        ['ZAngle', 'Returns the Z rotation angle of the given VuforiaTrackingResults object.'],
    ];
    this.setTooltip(function() {
      var key = thisBlock.getFieldValue('PROP');
      for (var i = 0; i < TOOLTIPS.length; i++) {
        if (TOOLTIPS[i][0] == key) {
          return TOOLTIPS[i][1];
        }
      }
      return '';
    });
    this.getFtcJavaOutputType = function() {
      var property = thisBlock.getFieldValue('PROP');
      switch (property) {
        case 'X':
        case 'Y':
        case 'Z':
        case 'XAngle':
        case 'YAngle':
        case 'ZAngle':
          return 'float';
        default:
          throw 'Unexpected property ' + property + ' (vuforiaTrackingResults_getProperty_Number getOutputType).';
      }
    };
  }
};

Blockly.JavaScript['vuforiaTrackingResults_getProperty_Number'] = function(block) {
  var property = block.getFieldValue('PROP');
  var vuforiaTrackingResults = Blockly.JavaScript.valueToCode(
      block, 'VUFORIA_TRACKING_RESULTS', Blockly.JavaScript.ORDER_MEMBER);
  var code = vuforiaTrackingResults + '.' + property;
  var blockLabel = 'VuforiaTrackingResults.' + block.getField('PROP').getText();
  return wrapJavaScriptCode(code, blockLabel);
};

Blockly.FtcJava['vuforiaTrackingResults_getProperty_Number'] = function(block) {
  var property = block.getFieldValue('PROP');
  var vuforiaTrackingResults = Blockly.FtcJava.valueToCode(
      block, 'VUFORIA_TRACKING_RESULTS', Blockly.FtcJava.ORDER_MEMBER);
  var code = vuforiaTrackingResults + '.' + Blockly.FtcJava.makeFirstLetterLowerCase_(property);
  return [code, Blockly.FtcJava.ORDER_MEMBER];
};

Blockly.Blocks['vuforiaTrackingResults_getUpdatedRobotLocation'] = {
  init: function() {
    this.setOutput(true, 'OpenGLMatrix');
    this.appendDummyInput()
        .appendField('call')
        .appendField(createNonEditableField('VuforiaTrackingResults'))
        .appendField('.')
        .appendField(createNonEditableField('getUpdatedRobotLocation'));
    this.appendValueInput('VUFORIA_TRACKING_RESULTS').setCheck('VuforiaBase.TrackingResults')
        .appendField('vuforiaTrackingResults')
        .setAlign(Blockly.ALIGN_RIGHT);
    this.setColour(functionColor);
    this.setTooltip('Returns a matrix representing the location of the robot.');
  }
};

Blockly.JavaScript['vuforiaTrackingResults_getUpdatedRobotLocation'] = function(block) {
  var vuforiaTrackingResults = Blockly.JavaScript.valueToCode(
      block, 'VUFORIA_TRACKING_RESULTS', Blockly.JavaScript.ORDER_MEMBER);
  var code = vuforiaTrackingResults + '.IsUpdatedRobotLocation ' +
      '? ' + miscIdentifierForJavaScript + '.getUpdatedRobotLocation(' +
          vuforiaTrackingResults + '.X, ' + vuforiaTrackingResults + '.Y, ' + vuforiaTrackingResults + '.Z, ' +
          vuforiaTrackingResults + '.XAngle, ' + vuforiaTrackingResults + '.YAngle, ' + vuforiaTrackingResults + '.ZAngle) ' +
      ': ' + miscIdentifierForJavaScript + '.getNull()';
  var blockLabel = 'VuforiaTrackingResults.getUpdatedRobotLocation';
  return wrapJavaScriptCode(code, blockLabel);
};

Blockly.FtcJava['vuforiaTrackingResults_getUpdatedRobotLocation'] = function(block) {
  var vuforiaTrackingResults = Blockly.FtcJava.valueToCode(
      block, 'VUFORIA_TRACKING_RESULTS', Blockly.FtcJava.ORDER_MEMBER);
  var code = vuforiaTrackingResults + '.isUpdatedRobotLocation ' +
      '? ' + vuforiaTrackingResults + '.matrix' +
      ': null';
  return [code, Blockly.FtcJava.ORDER_CONDITIONAL];
};

Blockly.Blocks['vuforiaTrackingResults_formatAsTransform'] = {
  init: function() {
    this.setOutput(true, 'String');
    this.appendDummyInput()
        .appendField('call')
        .appendField(createNonEditableField('VuforiaTrackingResults'))
        .appendField('.')
        .appendField(createNonEditableField('formatAsTransform'));
    this.appendValueInput('VUFORIA_TRACKING_RESULTS').setCheck('VuforiaBase.TrackingResults')
        .appendField('vuforiaTrackingResults')
        .setAlign(Blockly.ALIGN_RIGHT);
    this.setColour(functionColor);
    this.setTooltip('Returns a text representation of the given VuforiaTrackingResults.');
  }
};

Blockly.JavaScript['vuforiaTrackingResults_formatAsTransform'] = function(block) {
  var vuforiaTrackingResults = Blockly.JavaScript.valueToCode(
      block, 'VUFORIA_TRACKING_RESULTS', Blockly.JavaScript.ORDER_MEMBER);
  var code = '"{EXTRINSIC XYZ " + ' +
      vuforiaTrackingResults + '.XAngle.toFixed(0) + " " + ' +
      vuforiaTrackingResults + '.YAngle.toFixed(0) + " " + ' +
      vuforiaTrackingResults + '.ZAngle.toFixed(0) + "} {" + ' +
      vuforiaTrackingResults + '.X.toFixed(2) + " " + ' +
      vuforiaTrackingResults + '.Y.toFixed(2) + " " + ' +
      vuforiaTrackingResults + '.Z.toFixed(2) + "}"';
  var blockLabel = 'VuforiaTrackingResults.formatAsTransform';
  return wrapJavaScriptCode(code, blockLabel);
};

Blockly.FtcJava['vuforiaTrackingResults_formatAsTransform'] = function(block) {
  var vuforiaTrackingResults = Blockly.FtcJava.valueToCode(
      block, 'VUFORIA_TRACKING_RESULTS', Blockly.FtcJava.ORDER_MEMBER);
  var code = vuforiaTrackingResults + '.formatAsTransform()';
  return [code, Blockly.FtcJava.ORDER_FUNCTION_CALL];
};

Blockly.Blocks['vuforiaTrackingResults_toText'] = {
  init: function() {
    this.setOutput(true, 'String');
    this.appendDummyInput()
        .appendField('call')
        .appendField(createNonEditableField('VuforiaTrackingResults'))
        .appendField('.')
        .appendField(createNonEditableField('toText'));
    this.appendValueInput('VUFORIA_TRACKING_RESULTS').setCheck('VuforiaBase.TrackingResults')
        .appendField('vuforiaTrackingResults')
        .setAlign(Blockly.ALIGN_RIGHT);
    this.setColour(functionColor);
    this.setTooltip('Returns a text representation of the given VuforiaTrackingResults.');
  }
};

Blockly.JavaScript['vuforiaTrackingResults_toText'] = function(block) {
  var vuforiaTrackingResults = Blockly.JavaScript.valueToCode(
      block, 'VUFORIA_TRACKING_RESULTS', Blockly.JavaScript.ORDER_NONE);
  var code = 'JSON.stringify(' + vuforiaTrackingResults + ')';
  var blockLabel = 'VuforiaTrackingResults.toText';
  return wrapJavaScriptCode(code, blockLabel);
};

Blockly.FtcJava['vuforiaTrackingResults_toText'] = function(block) {
  var vuforiaTrackingResults = Blockly.FtcJava.valueToCode(
      block, 'VUFORIA_TRACKING_RESULTS', Blockly.FtcJava.ORDER_MEMBER);
  var code = vuforiaTrackingResults + '.toString()';
  return [code, Blockly.FtcJava.ORDER_FUNCTION_CALL];
};

// Previously in navigation.js:

// CameraDirection
Blockly.Blocks['navigation_enum_cameraDirection'] = {
  init: function() {
    var CAMERA_DIRECTION_CHOICES = [
        ['BACK', 'BACK'],
        ['FRONT', 'FRONT'],
    ];
    this.setOutput(true, 'VuforiaLocalizer.CameraDirection');
    this.appendDummyInput()
        .appendField(createNonEditableField('CameraDirection'))
        .appendField('.')
        .appendField(new Blockly.FieldDropdown(CAMERA_DIRECTION_CHOICES), 'CAMERA_DIRECTION');
    this.setColour(getPropertyColor);
    // Assign 'this' to a variable for use in the tooltip closure below.
    var thisBlock = this;
    var TOOLTIPS = [
        ['BACK', 'The CameraDirection value BACK.'],
        ['FRONT', 'The CameraDirection value FRONT.'],
    ];
    this.setTooltip(function() {
      var key = thisBlock.getFieldValue('CAMERA_DIRECTION');
      for (var i = 0; i < TOOLTIPS.length; i++) {
        if (TOOLTIPS[i][0] == key) {
          return TOOLTIPS[i][1];
        }
      }
      return '';
    });
  }
};

Blockly.JavaScript['navigation_enum_cameraDirection'] = function(block) {
  var code = '"' + block.getFieldValue('CAMERA_DIRECTION') + '"';
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};

Blockly.FtcJava['navigation_enum_cameraDirection'] = function(block) {
  var code = 'VuforiaLocalizer.CameraDirection.' + block.getFieldValue('CAMERA_DIRECTION');
  Blockly.FtcJava.generateImport_('VuforiaLocalizer');
  return [code, Blockly.FtcJava.ORDER_MEMBER];
};

Blockly.Blocks['navigation_typedEnum_cameraDirection'] =
    Blockly.Blocks['navigation_enum_cameraDirection'];

Blockly.JavaScript['navigation_typedEnum_cameraDirection'] =
    Blockly.JavaScript['navigation_enum_cameraDirection'];

Blockly.FtcJava['navigation_typedEnum_cameraDirection'] =
    Blockly.FtcJava['navigation_enum_cameraDirection'];

Blockly.Blocks['navigation_switchableCamera'] = {
  init: function() {
    this.setOutput(true, 'SwitchableCamera');
    this.appendDummyInput()
        .appendField(createNonEditableField('SwitchableCamera'));
    this.setColour(getPropertyColor);
    this.setTooltip('A virtual camera comprised of all configured webcams.');
  }
};

Blockly.JavaScript['navigation_switchableCamera'] = function(block) {
  // For javascript, we generate the special name of the switchable camera as a string.
  var code = '"' + switchableCameraName + '"';
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};

Blockly.FtcJava['navigation_switchableCamera'] = function(block) {
  // For java, we generate code to get a switchable camera.
  var code = 'VuforiaBase.getSwitchableCamera(hardwareMap)';
  Blockly.FtcJava.generateImport_('VuforiaBase');
  return [code, Blockly.FtcJava.ORDER_FUNCTION_CALL];
};

// CameraMonitorFeedback
Blockly.Blocks['navigation_enum_cameraMonitorFeedback'] = {
  init: function() {
    var CAMERA_MONITOR_FEEDBACK_CHOICES = [
        ['DEFAULT', 'DEFAULT'],
        ['NONE', 'NONE'],
        ['AXES', 'AXES'],
        ['TEAPOT', 'TEAPOT'],
        ['BUILDINGS', 'BUILDINGS'],
        ];
    this.setOutput(true, 'VuforiaLocalizer.Parameters.CameraMonitorFeedback');
    this.appendDummyInput()
        .appendField(createNonEditableField('CameraMonitorFeedback'))
        .appendField('.')
        .appendField(new Blockly.FieldDropdown(CAMERA_MONITOR_FEEDBACK_CHOICES), 'CAMERA_MONITOR_FEEDBACK');
    this.setColour(getPropertyColor);
    // Assign 'this' to a variable for use in the tooltip closure below.
    var thisBlock = this;
    var TOOLTIPS = [
        ['DEFAULT', 'The CameraMonitorFeedback value DEFAULT.'],
        ['NONE', 'The CameraMonitorFeedback value NONE.'],
        ['AXES', 'The CameraMonitorFeedback value AXES.'],
        ['TEAPOT', 'The CameraMonitorFeedback value TEAPOT.'],
        ['BUILDINGS', 'The CameraMonitorFeedback value BUILDINGS.'],
        ];
    this.setTooltip(function() {
      var key = thisBlock.getFieldValue('CAMERA_MONITOR_FEEDBACK');
      for (var i = 0; i < TOOLTIPS.length; i++) {
        if (TOOLTIPS[i][0] == key) {
          return TOOLTIPS[i][1];
        }
      }
      return '';
    });
  }
};

Blockly.JavaScript['navigation_enum_cameraMonitorFeedback'] = function(block) {
  var code = '"' + block.getFieldValue('CAMERA_MONITOR_FEEDBACK') + '"';
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};

Blockly.FtcJava['navigation_enum_cameraMonitorFeedback'] = function(block) {
  var cameraMonitorFeedback = block.getFieldValue('CAMERA_MONITOR_FEEDBACK');
  if (cameraMonitorFeedback == 'DEFAULT') {
    return ['null', Blockly.FtcJava.ORDER_ATOMIC];
  }
  var code = 'VuforiaLocalizer.Parameters.CameraMonitorFeedback.' + cameraMonitorFeedback;
  Blockly.FtcJava.generateImport_('VuforiaLocalizer');
  return [code, Blockly.FtcJava.ORDER_MEMBER];
};

Blockly.Blocks['navigation_typedEnum_cameraMonitorFeedback'] =
   Blockly.Blocks['navigation_enum_cameraMonitorFeedback'];

Blockly.JavaScript['navigation_typedEnum_cameraMonitorFeedback'] =
   Blockly.JavaScript['navigation_enum_cameraMonitorFeedback'];

Blockly.FtcJava['navigation_typedEnum_cameraMonitorFeedback'] =
   Blockly.FtcJava['navigation_enum_cameraMonitorFeedback'];
