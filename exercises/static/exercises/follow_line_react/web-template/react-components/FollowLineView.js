import React from 'react';
import classNames from 'classnames';
import './css/FollowLineView.css';

class FollowLineView extends React.Component {
  constructor(props) {
    super(props);
    this.canvasClass = classNames({ "follow-line-view": true });
  }

  render() {
    return <canvas className={this.canvasClass}></canvas>
  }
}

export default FollowLineView;