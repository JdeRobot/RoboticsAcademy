document.addEventListener('DOMContentLoaded', function() {
    const codeBlocks = document.querySelectorAll('pre');
  
    codeBlocks.forEach((codeBlock) => {
      const copyButton = document.createElement('button');
      copyButton.className = 'copy-code-button';
      copyButton.type = 'button';
      copyButton.innerHTML = '<i class="fas fa-copy"></i>';
      
      codeBlock.parentNode.style.position = 'relative';
      codeBlock.parentNode.appendChild(copyButton);
  
      copyButton.addEventListener('click', function() {
        const code = codeBlock.querySelector('code').innerText;
        navigator.clipboard.writeText(code).then(() => {
          copyButton.innerHTML = '<i class="fas fa-check"></i>';
          setTimeout(() => {
            copyButton.innerHTML = '<i class="fas fa-copy"></i>';
          }, 2000);
        });
      });
    });
  });