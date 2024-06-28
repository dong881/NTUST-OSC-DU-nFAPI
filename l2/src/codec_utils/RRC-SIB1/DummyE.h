/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_DummyE_H_
#define	_DummyE_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DummyE__maxNumberTxPortsPerResource {
	DummyE__maxNumberTxPortsPerResource_p4	= 0,
	DummyE__maxNumberTxPortsPerResource_p8	= 1,
	DummyE__maxNumberTxPortsPerResource_p12	= 2,
	DummyE__maxNumberTxPortsPerResource_p16	= 3,
	DummyE__maxNumberTxPortsPerResource_p24	= 4,
	DummyE__maxNumberTxPortsPerResource_p32	= 5
} e_DummyE__maxNumberTxPortsPerResource;
typedef enum DummyE__amplitudeScalingType {
	DummyE__amplitudeScalingType_wideband	= 0,
	DummyE__amplitudeScalingType_widebandAndSubband	= 1
} e_DummyE__amplitudeScalingType;

/* DummyE */
typedef struct DummyE {
	long	 maxNumberTxPortsPerResource;
	long	 maxNumberResources;
	long	 totalNumberTxPorts;
	long	 parameterLx;
	long	 amplitudeScalingType;
	long	 maxNumberCSI_RS_PerResourceSet;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DummyE_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_maxNumberTxPortsPerResource_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_amplitudeScalingType_12;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_DummyE;
extern asn_SEQUENCE_specifics_t asn_SPC_DummyE_specs_1;
extern asn_TYPE_member_t asn_MBR_DummyE_1[6];

#ifdef __cplusplus
}
#endif

#endif	/* _DummyE_H_ */
#include <asn_internal.h>
