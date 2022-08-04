import * as React from 'react';

const CustomComponent = (props) => {
  return (
      <div>
        <h1>THIS IS ANOTHER CUSTOM COMPONENT</h1>
        <div><span>prop1 = </span><span>{props.prop1}</span></div>
        <div><span>intprop = </span><span>{props.intprop}</span></div>
        <div><span>floatprop = </span><span>{props.floatprop}</span></div>
        <div><span>anotherprop = </span><span>{props.anotherprop}</span></div>
        {props.children}
      </div>
  )
};

CustomComponent.defaultProps = {
  prop1: "",
  intprop: 0,
  floatprop: 0.0,
  anotherprop: false
};

export default CustomComponent;