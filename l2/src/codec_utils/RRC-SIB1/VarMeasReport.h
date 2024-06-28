/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-UE-Variables"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_VarMeasReport_H_
#define	_VarMeasReport_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MeasId.h"
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct CellsTriggeredList;
struct CLI_TriggeredList_r16;
struct Tx_PoolMeasList_r16;
struct RelaysTriggeredList_r17;

/* VarMeasReport */
typedef struct VarMeasReport {
	MeasId_t	 measId;
	struct CellsTriggeredList	*cellsTriggeredList;	/* OPTIONAL */
	long	 numberOfReportsSent;
	struct CLI_TriggeredList_r16	*cli_TriggeredList_r16;	/* OPTIONAL */
	struct Tx_PoolMeasList_r16	*tx_PoolMeasToAddModListNR_r16;	/* OPTIONAL */
	struct RelaysTriggeredList_r17	*relaysTriggeredList_r17;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} VarMeasReport_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_VarMeasReport;
extern asn_SEQUENCE_specifics_t asn_SPC_VarMeasReport_specs_1;
extern asn_TYPE_member_t asn_MBR_VarMeasReport_1[6];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "CellsTriggeredList.h"
#include "CLI-TriggeredList-r16.h"
#include "Tx-PoolMeasList-r16.h"
#include "RelaysTriggeredList-r17.h"

#endif	/* _VarMeasReport_H_ */
#include <asn_internal.h>
