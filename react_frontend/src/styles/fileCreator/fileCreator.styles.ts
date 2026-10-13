import styled from "styled-components";

const primaryColor = "#666";

interface StyledColor {
  color?: string;
}

export const StyledTemplatesTitle = styled.summary<StyledColor>`
  width: 100%;
  border: none;
  font-size: large;
  font-weight: bold;
  text-align: center;
  margin: 10px 0px;
  color: ${(p) => p.color ?? primaryColor};
`;
