import styled from "styled-components";

const primaryColor = "#666";

interface StyledHeaderTextProps {
  color?: string;
}

export const StyledHeaderText = styled.h1<StyledHeaderTextProps>`
  font-size: 25px;
  margin: 0;
  color: ${(p) => p.color ?? primaryColor};
`;

interface StyledProjectProps {
  color?: string;
}

export const StyledProject = styled.span<StyledProjectProps>`
  display: flex;
  flex-direction: column;
  background-color: none;
  color: ${(p) => p.color ?? primaryColor};
  padding: 10px;
  justify-content: center;
  margin-left: auto;

  div {
    font-weight: bold;
  }
`;
