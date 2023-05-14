import classNames from "classnames";

import '../css/FlexContainer.css';

const FlexContainer = (props) => {
  var containerClass = classNames({
    'flex-container': true,
    'flex-container-row': props.row,
    'flex-container-column': !props.row
  });

  var separatorClass = classNames({
    'fa': true,
    'fa-ellipsis-v': props.row,
    'fa-ellipsis-h': !props.row
  });

  return (
    <div className={containerClass}>
      <div className={"flex-container-first"}>{props.children[0]}</div>
      <div className={"flex-container-divider"}>
        <i className={separatorClass}></i>
      </div>
      <div className={"flex-container-last"}>{props.children.slice(1)}</div>
    </div>
  );
};

export default FlexContainer;