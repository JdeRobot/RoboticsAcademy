import { useState, useEffect } from "react";

export const useTabLock = (project: string) => {
  const [isLocked, setIsLocked] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    const lockKey = `tab_lock_${project}`;
    const myId =
      Math.random().toString(36).substring(2) + Date.now().toString(36);
    const heartbeatInterval = 1000;
    const lockTimeout = 3000;

    let heartbeatTimer: NodeJS.Timeout;
    let checkTimer: NodeJS.Timeout;

    const checkLock = () => {
      const lockData = localStorage.getItem(lockKey);
      const now = Date.now();

      if (lockData) {
        const { id, timestamp } = JSON.parse(lockData);

        if (id === myId) {
          // We already own the lock, update local state if needed
          setIsLocked(false);
          setIsLoading(false);
          // Ensure heartbeat is running if it stopped (though unlikely with this logic flow)
          if (!heartbeatTimer) {
             startHeartbeat();
          }
          return;
        }

        if (now - timestamp < lockTimeout) {
          // Locked by someone else
          setIsLocked(true);
          setIsLoading(false);
          return;
        }
      }

      // Try to acquire lock
      tryAcquireLock();
    };

    const tryAcquireLock = () => {
      // Optimistic write
      const now = Date.now();
      const lockData = { id: myId, timestamp: now };
      localStorage.setItem(lockKey, JSON.stringify(lockData));

      // Verify after a tiny delay to resolve races
      setTimeout(() => {
        const currentLock = localStorage.getItem(lockKey);
        if (currentLock) {
          const { id } = JSON.parse(currentLock);
          if (id === myId) {
            // We won
            setIsLocked(false);
            setIsLoading(false);
            startHeartbeat();
          } else {
            // We lost to a faster writer
            setIsLocked(true);
            setIsLoading(false);
          }
        }
      }, 50);
    };

    const startHeartbeat = () => {
      heartbeatTimer = setInterval(() => {
        const lockData = { id: myId, timestamp: Date.now() };
        localStorage.setItem(lockKey, JSON.stringify(lockData));
      }, heartbeatInterval);
    };

    // Initial check
    checkTimer = setInterval(checkLock, 2000); // Poll to see if lock frees up
    checkLock(); // Immediate check

    // Listener for storage events (faster reaction to lock changes)
    const handleStorage = (e: StorageEvent) => {
      if (e.key === lockKey) {
        if (!e.newValue) {
          // Lock cleared, try to acquire
          checkLock();
        } else {
          // Lock updated, check if it's us or someone else
          const { id } = JSON.parse(e.newValue);
          if (id !== myId) {
            setIsLocked(true);
          }
        }
      }
    };
    window.addEventListener("storage", handleStorage);

    return () => {
      clearInterval(heartbeatTimer);
      clearInterval(checkTimer);
      window.removeEventListener("storage", handleStorage);

      // Release lock if we own it
      const currentLock = localStorage.getItem(lockKey);
      if (currentLock) {
        const { id } = JSON.parse(currentLock);
        if (id === myId) {
          localStorage.removeItem(lockKey);
        }
      }
    };
  }, [project]);

  return { isLocked, isLoading };
};
