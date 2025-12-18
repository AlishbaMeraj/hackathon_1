import React from 'react';
import clsx from 'clsx';
import styles from './IsaacConceptCard.module.css';

/**
 * Component for displaying NVIDIA Isaac concepts in a card format
 */
export default function IsaacConceptCard({title, description, icon}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}